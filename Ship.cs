using Godot;
using Godot.Collections;
using System;
using System.Collections.Generic;
using System.Linq;
using Tensorflow;
using Tensorflow.Gradients;
using Tensorflow.Keras.Engine;
using Tensorflow.Keras.Losses;
using Tensorflow.Keras.Optimizers;
using Tensorflow.NumPy;
using Tensorflow.Operations.Losses;
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

    public IActionSelector ActionSelector = new PlayerActionSelector();
    public Action<State, Action, float, State, bool> SaveExperience;
    public Func<Sequential> QNetworkFunc;
    private Sequential QNetwork => QNetworkFunc();
    public Func<Sequential> TargetQNetworkFunc;
    private Sequential TargetQNetwork => TargetQNetworkFunc();
    public Func<FixedSizeList<Experience>> ExperienceFun;
    private FixedSizeList<Experience> Experiences => ExperienceFun();


    private int NUM_STEPS_FOR_UPDATE = 4;
    private int UPDATE_BATCH_SIZE = 64;
    private float DISCOUNT_FACTOR = 0.995f;
    private float SOFT_UPDATE_FACTOR = 0.001f;

    private Sprite2D _leftThruster;
    private Sprite2D _rightThruster;
    private Timer _maxMissionLengthTimer;

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
            EmitSignal(SignalName.SimulationEnd);
            _simulationInProgress = false;
        }
    }

    private void ApplyReward(float value)
    {
        if (_simulationInProgress)
        {
            _rewardSinceLastStep += value;
            EmitSignal(SignalName.ScoreChange, value);
        }
    }

    private void Learn()
    {
        _learnStepCount++;

        if (_learnStepCount % NUM_STEPS_FOR_UPDATE != 0
            || Experiences.Count < UPDATE_BATCH_SIZE)
        {
            return;
        }

        var optimizer = new Adam(learning_rate: 0.001f);

        using (var g = tf.GradientTape())
        {
            var loss = CalculateLoss(Experiences.Get(UPDATE_BATCH_SIZE), DISCOUNT_FACTOR, QNetwork, TargetQNetwork);

            // Get the gradients of the loss with respect to the weights.
            var gradients = g.gradient(loss, QNetwork.Weights);

            // Update the weights of the q_network.
            optimizer.apply_gradients(zip(gradients, QNetwork.TrainableVariables.Select(x => x as ResourceVariable)));

            // Update the weights of target q_network
            foreach (var (qWeights, targetQWeights) in zip(QNetwork.Weights, TargetQNetwork.Weights))
            {
                targetQWeights.assign(
                    SOFT_UPDATE_FACTOR * qWeights.AsTensor() 
                    + ((1.0 - SOFT_UPDATE_FACTOR) * targetQWeights.AsTensor()));
            }
        }
    }

    private static Tensor CalculateLoss(List<Experience> experiences, float gamma, Sequential qNetwork, Sequential targetQNetwork)
    {
        NDArray states = new(experiences.Select(e => e.State).ToArray());
        states = states.reshape((-1, STATE_SIZE));

        NDArray rewards = new(experiences.Select(e => e.Reward).ToArray());

        NDArray nextStates = new(experiences.Select(e => e.NextState).ToArray());
        nextStates = nextStates.reshape((-1, STATE_SIZE));

        NDArray done = new(experiences.Select(e => e.Ended ? 1 : 0).ToArray());

        var maxQSA = tf.reduce_max(targetQNetwork.predict(nextStates));

        var yTargets = rewards + ((1 - done) * gamma * maxQSA);

        var qValues = qNetwork.predict(states);

        List<float> selectedQValues = new();
        int i = 0;
        foreach (Action action in experiences.Select(e => e.Action))
        {
            selectedQValues.Add(((float)qValues[0][i][(int)action]));
        }

        Tensor tensorQValues = new(selectedQValues.ToArray());

        // Calculated MSE loss
        var loss = tf.reduce_sum(tf.pow(tensorQValues - yTargets, 2)) / (2f * yTargets.shape[0]);

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
                Tensor prediction = QNetwork.predict(state);
                var bestActionArg = np.argmax(prediction[0][0].numpy());
                return GetActionFromIndex(bestActionArg);
            }
            else // Random action
            {
                int rand = _rng.RandiRange(0, 3);
                return GetActionFromIndex(rand);
            }
        }

        private static Action GetActionFromIndex(int ind)
        {
            switch (ind)
            {
                case 0:
                    return Action.Nothing;
                case 1:
                    return Action.ThrustDown;
                case 2:
                    return Action.ThrustLeft;
                case 3:
                    return Action.ThrustRight;
                default:
                    return Action.Nothing;
            }
        }
    }
}



