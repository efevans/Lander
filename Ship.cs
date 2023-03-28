using Godot;
using Godot.Collections;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection.Metadata;
using Tensorflow;
using Tensorflow.Gradients;
using Tensorflow.Keras.Engine;
using Tensorflow.Keras.Losses;
using Tensorflow.Keras.Optimizers;
using Tensorflow.NumPy;
using Tensorflow.Operations.Losses;
using static Ship;
using static Tensorflow.Binding;
using static Tensorflow.KerasApi;
using static World;

public partial class Ship : RigidBody2D
{
    [Signal]
    public delegate void ScoreChangeEventHandler(float val);
    [Signal]
    public delegate void SimulationEndEventHandler(float val);

    [Export]
    public int DownwardThrust = 300;
    [Export]
    public float DownwardThrustReward;
    [Export]
    public int SideThrust = 100;
    [Export]
    public float SideThrustReward;
    [Export]
    public float WinReward;
    [Export]
    public float CrashReward;
    [Export]
    public float FirstLegTouchReward;
    [Export]
    public float ExceededMaxMissionLengthReward;

    private Sequential _qNetwork;
    private Sequential _targetQNetwork;
    public const int STATE_SIZE = 8;
    private const int ACTION_SIZE = 4;
    private const float EPSILON_MIN = 0.01f;
    private const float EPSILON_DECAY = 0.995f;
    private float _epsilon = 1;

    private int NUM_STEPS_FOR_UPDATE = 4;
    private int UPDATE_BATCH_SIZE = 64;
    private float DISCOUNT_FACTOR = 0.995f;
    private float SOFT_UPDATE_FACTOR = 0.001f;
    private const int BUFFER_SIZE = 100000;
    private readonly FixedSizeExperienceList _experience = new(BUFFER_SIZE);

    public IActionSelector ActionSelector = new PlayerActionSelector();
    private Sprite2D _leftThruster;
    private Sprite2D _rightThruster;
    private Timer _maxMissionLengthTimer;
    private RandomNumberGenerator _rng;

    private bool _resetNextTick = false;
    private ResetOption _resetOption;
    private struct ResetOption
    {
        public Vector2 Position;
        public Vector2 Velocity;
        public float Rotation;
        public float AngularVelocity;
    }

    public void Reset(Vector2 position, Vector2 velocity, float rotation, float angularVelocity)
    {
        _resetOption = new ResetOption { Position = position, Velocity = velocity, Rotation = rotation, AngularVelocity = angularVelocity };
        _resetNextTick = true;
        _maxMissionLengthTimer.Stop();
        _maxMissionLengthTimer.Start();

        // Setting immediately can cause the current simulation to set "In progress" to false again, so 
        // defer it to ensure we the current physics steps moves up back to reset position first
        SetDeferred("_simulationInProgress", true);
    }

    public override void _IntegrateForces(PhysicsDirectBodyState2D state)
    {
        if (_resetNextTick)
        {
            _firstStepOfSimulation = true;
            _resetNextTick = false;
            _simulationEndedLastStep = false;
            _rewardSinceLastStep = 0;
            _learnStepCount = 0;

            _anyLegHasTouched = false;
            _leftLegTouching = false;
            _rightLegTouching = false;

            var transform = state.Transform;
            transform.Origin.X = _resetOption.Position.X;
            transform.Origin.Y = _resetOption.Position.Y;
            transform = transform.RotatedLocal(_resetOption.Rotation);
            state.Transform = transform;
            LinearVelocity = _resetOption.Velocity;
            AngularVelocity = _resetOption.AngularVelocity;
        }
    }

    // Called when the node enters the scene tree for the first time.
    public override void _Ready()
    {
        _leftThruster = GetNode<Sprite2D>("ThrusterLeftSprite");
        _rightThruster = GetNode<Sprite2D>("ThrusterRightSprite");
        _maxMissionLengthTimer = GetNode<Timer>("MaxMissionLengthTimer");
        _rng = new RandomNumberGenerator();

        ActionSelector = new DQLActionSelector(_epsilon, _qNetwork, _rng);
    }

    public override void _PhysicsProcess(double delta)
    {
        State currState = ToState();

        if (false == _firstStepOfSimulation)
        {
            SaveExperience(_lastState, _lastAction, _rewardSinceLastStep, currState, _simulationEndedLastStep);
        }

        _firstStepOfSimulation = false;
        _simulationEndedLastStep = false;
        _rewardSinceLastStep = 0;

        Action action = ActionSelector.GetNextAction(this);

        // Use else-if here to restrict our action space to a size of 4
        if (action == Action.ThrustDown)
        {
            var translatedForce = Transform.BasisXform(Vector2.Up * DownwardThrust);
            ApplyForce(translatedForce);
            ApplyReward(DownwardThrustReward);
        }
        else if (action == Action.ThrustRight)
        {
            var translatedPosition = Transform.BasisXform(_rightThruster.Position);
            var translatedForce = Transform.BasisXform(Vector2.Left * SideThrust);
            ApplyForce(translatedForce, translatedPosition);
            ApplyReward(SideThrustReward);
        }
        else if (action == Action.ThrustLeft)
        {
            var translatedPosition = Transform.BasisXform(_leftThruster.Position);
            var translatedForce = Transform.BasisXform(Vector2.Right * SideThrust);
            ApplyForce(translatedForce, translatedPosition);
            ApplyReward(SideThrustReward);
        }

        if (_leftLegTouching && _rightLegTouching)
        {
            if (_wasTouchingDownLastFrame)
            {
                _touchdownDownDuration = _touchdownDownDuration.Add(new TimeSpan(0, 0, 0, 0, (int)(delta * 1000f)));
                if (_touchdownDownDuration > _requiredTouchdownDurationForWin)
                {
                    GD.Print("Won!");
                    ApplyReward(WinReward);
                    EndSimulation();
                }
            }
            else
            {
                _wasTouchingDownLastFrame = true;
                _touchdownDownDuration = new TimeSpan(0);
            }
        }
        else
        {
            _wasTouchingDownLastFrame = false;
        }

        Learn();

        _lastState = currState;
        _lastAction = action;
    }

    // Store these values from a previous step to allow us to evaluate how well it did
    private Action _lastAction;
    private State _lastState;
    private float _rewardSinceLastStep;

    // State to track progress on rewards
    private int _learnStepCount = 0;
    private bool _firstStepOfSimulation = true;
    private bool _anyLegHasTouched = false;
    private bool _leftLegTouching = false;
    private bool _rightLegTouching = false;
    private bool _wasTouchingDownLastFrame = false;
    private TimeSpan _touchdownDownDuration;
    private readonly TimeSpan _requiredTouchdownDurationForWin = new(0, 0, 1);
    private bool _simulationEndedLastStep = false; // For calculating Q prime
    private bool _simulationInProgress = false;

    private System.Collections.Generic.Dictionary<int, CollisionPolygon2D> CollisionObjectsById { get; set; } = new System.Collections.Generic.Dictionary<int, CollisionPolygon2D>();

    private void OnBodyShapeEntered(Rid _, Godot.Node _2, long _3, long local_shape_index)
    {
        CollisionPolygon2D localCollisionObject = GetLocalCollisionObject(local_shape_index);
        GD.Print($"Found leg {localCollisionObject.Name} Landing");

        if (localCollisionObject.Name.ToString().Contains("Leg"))
        {
            if (false == _anyLegHasTouched)
            {
                _anyLegHasTouched = true;
                ApplyReward(FirstLegTouchReward);
            }

            if (localCollisionObject.Name.ToString().Contains("Left"))
            {
                _leftLegTouching = true;
            }
            else
            {
                _rightLegTouching = true;
            }

        }
        else if (localCollisionObject.Name.ToString().Contains("Body"))
        {
            GD.Print("CRASHED!");
            ApplyReward(CrashReward);
            EndSimulation();
        }
    }

    private void OnBodyShapeExited(Rid _, Godot.Node _2, long _3, long local_shape_index)
    {
        CollisionPolygon2D localCollisionObject = GetLocalCollisionObject(local_shape_index);
        GD.Print($"Found leg {localCollisionObject.Name} Taking off");

        if (localCollisionObject.Name.ToString().Contains("Leg"))
        {
            if (localCollisionObject.Name.ToString().Contains("Left"))
            {
                _leftLegTouching = false;
            }
            else
            {
                _rightLegTouching = false;
            }
        }
    }

    private CollisionPolygon2D GetLocalCollisionObject(long shape_index)
    {
        int shapeIndexInt = (int)shape_index;

        if (CollisionObjectsById.TryGetValue(shapeIndexInt, out CollisionPolygon2D collisionObject))
        {
            return collisionObject;
        }

        var local_shape_owner = ShapeFindOwner(shapeIndexInt);
        collisionObject = (CollisionPolygon2D)ShapeOwnerGetOwner(local_shape_owner);

        CollisionObjectsById[shapeIndexInt] = collisionObject;

        return collisionObject;
    }

    private void OnMaxMissionLengthTimer()
    {
        ApplyReward(ExceededMaxMissionLengthReward);
        EndSimulation();
        GD.Print("Max Mission Lenght Exceeded");
    }

    private void EndSimulation()
    {
        if (_simulationInProgress)
        {
            _simulationEndedLastStep = true;
            _epsilon = UpdateEpsilon(_epsilon);
            EmitSignal(SignalName.SimulationEnd);
            _simulationInProgress = false;
        }
    }

    private static float UpdateEpsilon(float epsilon)
    {
        return Math.Max(EPSILON_MIN, EPSILON_DECAY * epsilon);
    }

    private void ApplyReward(float value)
    {
        if (_simulationInProgress)
        {
            _rewardSinceLastStep += value;
            EmitSignal(SignalName.ScoreChange, value);
        }
    }

    private void InitNeuralNetworks()
    {
        (_qNetwork, _targetQNetwork) = GetDefaultQNetworks();
    }

    private (Sequential q_network, Sequential target_q_network) GetDefaultQNetworks()
    {
        var q_network = keras.Sequential();
        q_network.add(keras.layers.Input(shape: STATE_SIZE));
        q_network.add(keras.layers.Dense(64, activation: keras.activations.Relu));
        q_network.add(keras.layers.Dense(64, activation: keras.activations.Relu));
        q_network.add(keras.layers.Dense(ACTION_SIZE, activation: keras.activations.Linear));

        q_network.compile(optimizer: keras.optimizers.Adam(learning_rate: 0.002f),
            loss: keras.losses.MeanSquaredError(),
            System.Array.Empty<string>());

        var target_q_network = keras.Sequential();
        target_q_network.add(keras.layers.Input(shape: STATE_SIZE));
        target_q_network.add(keras.layers.Dense(64, activation: keras.activations.Relu));
        target_q_network.add(keras.layers.Dense(64, activation: keras.activations.Relu));
        target_q_network.add(keras.layers.Dense(ACTION_SIZE, activation: keras.activations.Linear));

        target_q_network.compile(optimizer: keras.optimizers.Adam(learning_rate: 0.002f),
            loss: keras.losses.MeanSquaredError(),
            System.Array.Empty<string>());

        for (int i = 0; i < q_network.Weights.Count; i++)
        {
            tf.assign(target_q_network.Weights[i], q_network.Weights[i]);
        }

        return (q_network, target_q_network);
    }

    private void Learn()
    {
        _learnStepCount++;

        if (_learnStepCount % NUM_STEPS_FOR_UPDATE != 0
            || _experience.Count < UPDATE_BATCH_SIZE)
        {
            return;
        }

        var optimizer = new Adam(learning_rate: 0.001f);

        (var states, var actions, var rewards, var nextStates, var done) = _experience.Get(UPDATE_BATCH_SIZE);

        using var g = tf.GradientTape(true);

        var loss = CalculateLoss(states, actions, rewards, nextStates, done, DISCOUNT_FACTOR, _qNetwork, _targetQNetwork);

        // Compute gradients
        var gradients = g.gradient(loss, _qNetwork.TrainableVariables);

        // Update the weights of the q_network.
        var zipped = zip(gradients, _qNetwork.TrainableVariables.Select(x => x as ResourceVariable));
        optimizer.apply_gradients(zipped);

        // Update the weights of target q_network
        foreach (var (qWeights, targetQWeights) in zip(_qNetwork.Weights, _targetQNetwork.Weights))
        {
            targetQWeights.assign(
                SOFT_UPDATE_FACTOR * qWeights.AsTensor()
                + ((1.0 - SOFT_UPDATE_FACTOR) * targetQWeights.AsTensor()));
        }
    }

    private static Tensor CalculateLoss(Tensor states, Tensor actions, Tensor rewards,
        Tensor nextStates, Tensor done, float gamma, Sequential qNetwork, Sequential targetQNetwork)
    {
        var maxQSA = tf.reduce_max(targetQNetwork.Apply(nextStates));

        var yTargets = (rewards + ((1 - done) * gamma * maxQSA)).numpy();

        var qValues = qNetwork.Apply(states);
        
        List<float> selectedQValues = new();
        int i = 0;
        foreach (var action in tf.unstack(actions))
        {
            selectedQValues.Add((float)qValues[0][i][(int)action]);
        }
        

        Tensor ndQValues = new(selectedQValues.ToArray());

        // Calculated MSE loss
        var loss = tf.reduce_sum(tf.pow(ndQValues - yTargets, 2)) / (2f * yTargets.shape[0]);

        return loss;
    }

    public State ToState()
    {
        return new State(this);
    }

    public class State
    {
        public State(Ship ship)
        {
            X = ship.Position.X;
            Y = ship.Position.Y;
            XVelocity = ship.LinearVelocity.X;
            YVelocity = ship.LinearVelocity.Y;
            Angle = ship.Rotation;
            AngularVelocity = ship.AngularVelocity;
            LeftLegTouching = ship._leftLegTouching;
            RightLegTouching = ship._rightLegTouching;
        }

        public NDArray ToNDArray()
        {
            NDArray arr = new(new float[] { X, Y, XVelocity, YVelocity, Angle, AngularVelocity, LeftLegTouching ? 1f : 0f, RightLegTouching ? 1f : 0f });
            arr = arr.astype(TF_DataType.TF_FLOAT);
            return arr.reshape((1, 8));
        }

        public float X;
        public float Y;
        public float XVelocity;
        public float YVelocity;
        public float Angle;
        public float AngularVelocity;
        public bool LeftLegTouching;
        public bool RightLegTouching;
    }

    public enum Action
    {
        Nothing = 0,
        ThrustDown = 1,
        ThrustLeft = 2,
        ThrustRight = 3
    }

    public interface IActionSelector
    {
        Action GetNextAction(Ship ship);
    }

    public class PlayerActionSelector : IActionSelector
    {
        public Action GetNextAction(Ship _)
        {
            Action action;

            // Use else-if here to restrict our action space to a size of 4
            if (Input.IsActionPressed("thrust_down"))
            {
                action = Action.ThrustDown;
            }
            else if (Input.IsActionPressed("thrust_right"))
            {
                action = Action.ThrustRight;
            }
            else if (Input.IsActionPressed("thrust_left"))
            {
                action = Action.ThrustLeft;
            }
            else
            {
                action = Action.Nothing;
            }

            return action;
        }
    }

    public class DQLActionSelector : IActionSelector
    {
        public float Epsilon;
        public Sequential QNetwork;
        private RandomNumberGenerator _rng;

        public DQLActionSelector(float epsilon, Sequential qNetwork, RandomNumberGenerator rng)
        {
            Epsilon = epsilon;
            QNetwork = qNetwork;
            _rng = rng;
        }

        public Action GetNextAction(Ship ship)
        {
            if (_rng.Randf() > Epsilon) // Pick best action from Q-Network
            {
                NDArray state = ship.ToState().ToNDArray();
                Tensor prediction = QNetwork.Apply(state);
                var bestActionArg = np.argmax(prediction[0].numpy());
                return (Action)(int)bestActionArg;
            }
            else // Random action
            {
                int rand = _rng.RandiRange(0, 3);
                return (Action)rand;
            }
        }
    }

    public class Experience
    {
        public Experience(State state, Ship.Action action, float reward, State nextState, bool ended)
        {
            State = state;
            NextState = nextState;
            Reward = reward;
            Action = action;
            Ended = ended;
        }

        public State State;
        public Ship.Action Action;
        public float Reward;
        public State NextState;
        public bool Ended;
    }

    public class FixedSizeExperienceList
    {
        private readonly List<Experience> _list = new List<Experience>();
        private int _limit;

        public int Count => _list.Count;

        public FixedSizeExperienceList(int limit)
        {
            _limit = limit;
        }

        public void Add(Experience obj)
        {
            _list.add(obj);
            if (_list.Count > _limit)
            {

            }
            while (_list.Count > _limit)
            {
                _list.RemoveAt(_list.Count - 1);
            }
        }

        // Returns Tensors containing, in order, current state, action, reward, next state, and done-ness
        // state and next state tensors have already been reshaped
        public (Tensor, Tensor, Tensor, Tensor, Tensor) Get(int count, RandomNumberGenerator rng = null)
        {
            HashSet<int> indecies = new HashSet<int>();
            if (rng == null)
            {
                rng = new RandomNumberGenerator();
            }

            for (int i = 0; i < count; i++)
            {
                int index = rng.RandiRange(0, _list.Count - 1);
                while (true == indecies.Contains(index))
                {
                    index = rng.RandiRange(0, _list.Count - 1);
                }
                indecies.Add(index);
            }

            List<Experience> chosenExperiences = indecies.Select(i => _list[i]).ToList();

            Tensor states = new(chosenExperiences.Select(e => e.State).Select(s => new float[] { s.X, s.Y, s.XVelocity, s.YVelocity, s.Angle, s.AngularVelocity, s.LeftLegTouching ? 1f : 0f, s.RightLegTouching ? 1f : 0f }).SelectMany(d => d).ToArray());
            states = tf.reshape(states, (-1, STATE_SIZE));

            Tensor actions = new(chosenExperiences.Select(e => (int)e.Action).ToArray());

            Tensor rewards = new(chosenExperiences.Select(e => e.Reward).ToArray());

            Tensor nextStates = new(chosenExperiences.Select(e => e.NextState).Select(s => new float[] { s.X, s.Y, s.XVelocity, s.YVelocity, s.Angle, s.AngularVelocity, s.LeftLegTouching ? 1f : 0f, s.RightLegTouching ? 1f : 0f }).SelectMany(d => d).ToArray());
            nextStates = tf.reshape(nextStates, (-1, STATE_SIZE));

            Tensor done = new(chosenExperiences.Select(e => e.Ended ? 1 : 0).ToArray());

            return (states, actions, rewards, nextStates, done);
        }
    }

    public void SaveExperience(State state, Ship.Action action, float reward, State nextState, bool ended)
    {
        var exp = new Experience(state, action, reward, nextState, ended);
        _experience.Add(exp);
    }
}



