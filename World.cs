using Godot;
using System;
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

        ResetShip();
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
}
