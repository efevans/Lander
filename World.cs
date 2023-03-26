using Godot;
using System.Collections.Generic;
using System.Linq;
using Tensorflow;
using Tensorflow.NumPy;
using static Ship;
using static Tensorflow.Binding;
using static Tensorflow.KerasApi;

using Sequential = Tensorflow.Keras.Engine.Sequential;

public partial class World : Node
{
    [Export]
	public int MaxStartVelocity;
	[Export]
	public float MaxStartAngularVelocity;

	private Ship _ship;
	private Marker2D _startPosition;
	private RandomNumberGenerator _rng;
	private Hud _hud;

	private Sequential _qNetwork;
	private Sequential _targetQNetwork;
    public const int STATE_SIZE = 8;
    private const int ACTION_SIZE = 4;
    private const int BUFFER_SIZE = 100000;
    private float _epsilon = 1;
    private readonly FixedSizeList<Experience> _experience = new(BUFFER_SIZE);
    private float _score = 0;
	private float Score 
	{ 
		get
		{
			return _score;
		}
		set 
		{
			_score = value;
			_hud.UpdateScore(_score);
		}
	}

    // Called when the node enters the scene tree for the first time.
    public override void _Ready()
	{
        _rng = new RandomNumberGenerator();
        _ship = GetNode<Ship>("Ship");
        _startPosition = GetNode<Marker2D>("StartPosition");
		_hud = GetNode<Hud>("HUD");

        _ship.SaveExperience = SaveExperience;
        _ship.QNetworkFunc = () => { return _qNetwork; };
        _ship.TargetQNetworkFunc = () => { return _targetQNetwork; };
        _ship.ExperienceFun = () => { return _experience; };
		InitNeuralNetworks();

        ResetShip();
    }

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
		GD.Print(Score);
	}

	private void ResetShip()
    {
        Score = 0;

        Vector2 position = _startPosition.Position;
		Vector2 velocity = new(_rng.RandfRange(-1f, 1f), _rng.RandfRange(-1f, 1f));
		velocity = velocity.Normalized();
		velocity *= _rng.RandiRange(0, MaxStartVelocity);
		float angle = _rng.RandfRange(-Mathf.Pi, Mathf.Pi);
		float angularVelocity = _rng.RandfRange(-MaxStartAngularVelocity, MaxStartAngularVelocity);

		_ship.Reset(position, velocity, angle, angularVelocity);
    }

	private void OnScoreChange(float score)
	{
        Score += score;
	}

	private void OnSimulationEnd()
    {
		GD.Print("SimulationEnd signal received");
		GD.Print($"End score was: {Score}");
        ResetShip();
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

    public void SaveExperience(State state, Action action, float reward, State nextState, bool ended)
    {
        var exp = new Experience(state, action, reward, nextState, ended);
        _experience.Add(exp);
    }

    public class Experience
    {
        public Experience(State state, Action action, float reward, State nextState, bool ended)
        {
            State = state;
            NextState = nextState;
            Reward = reward;
            Action = action;
            Ended = ended;
        }

        public State State;
        public Action Action;
        public float Reward;
        public State NextState;
        public bool Ended;
    }

    public class FixedSizeList<T>
    {
        private readonly List<T> _list = new List<T>();
        private int _limit;

        public int Count => _list.Count;

        public FixedSizeList(int limit)
        {
            _limit = limit;
        }

        public void Add(T obj)
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

        public List<T> Get(int count, RandomNumberGenerator rng = null)
        {
            List<T> values = new List<T>();
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

            return indecies.Select(i => _list[i]).ToList();
        }
    }
}
