using Godot;
using System;

public partial class World : Node
{
	[Export]
	public int MaxStartVelocity;
	[Export]
	public float MaxStartAngularVelocity;

	private Ship _ship;
	private Marker2D _startPosition;
	private RandomNumberGenerator _rng;

    // Called when the node enters the scene tree for the first time.
    public override void _Ready()
	{
        _rng = new RandomNumberGenerator();
        _ship = GetNode<Ship>("Ship");
        _startPosition = GetNode<Marker2D>("StartPosition");
		GetNode<Timer>("Timer").Start();
    }

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
	}

	private void ResetShip()
	{
		Vector2 position = _startPosition.Position;
		Vector2 velocity = new Vector2(_rng.RandfRange(-1f, 1f), _rng.RandfRange(-1f, 1f));
		velocity = velocity.Normalized();
		velocity *= _rng.RandiRange(0, MaxStartVelocity);
		float angle = _rng.RandfRange(-Mathf.Pi, Mathf.Pi);
		float angularVelocity = _rng.RandfRange(-MaxStartAngularVelocity, MaxStartAngularVelocity);

		_ship.Reset(position, velocity, angle, angularVelocity);
    }

    private void OnTimerTimeout()
	{
		ResetShip();
	}
}
