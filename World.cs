using Godot;
using System.Collections.Generic;
using Tensorflow;
using Tensorflow.NumPy;
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
    private const int STATE_SIZE = 8;
    private const int ACTION_SIZE = 4;
    private float _epsilon = 1;
	private Queue<Ship.StateActionPair> _stateActions;
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
}
