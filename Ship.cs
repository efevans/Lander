using Godot;
using Godot.Collections;
using System;

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

    }

    public override void _IntegrateForces(PhysicsDirectBodyState2D state)
    {
        if (_resetNextTick)
        {
            _resetNextTick = false;
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
        if (Input.IsActionPressed("thrust_down"))
        {
            var translatedForce = Transform.BasisXform(Vector2.Up * DownwardThrust);
            ApplyForce(translatedForce);
            EmitSignal(SignalName.ScoreChange, DownwardThrustReward);
        }

        if (Input.IsActionPressed("thrust_right"))
        {
            var translatedPosition = Transform.BasisXform(_rightThruster.Position);
            var translatedForce = Transform.BasisXform(Vector2.Left * SideThrust);
            ApplyForce(translatedForce, translatedPosition);
            EmitSignal(SignalName.ScoreChange, SideThrustReward);
        }

        if (Input.IsActionPressed("thrust_left"))
        {
            var translatedPosition = Transform.BasisXform(_leftThruster.Position);
            var translatedForce = Transform.BasisXform(Vector2.Right * SideThrust);
            ApplyForce(translatedForce, translatedPosition);
            EmitSignal(SignalName.ScoreChange, SideThrustReward);
        }

        if (_numberOfTouchingLegs == 2)
        {
            if (_wasTouchingDownLastFrame)
            {
                _touchdownDownDuration = _touchdownDownDuration.Add(new TimeSpan(0, 0, 0, 0, (int)(delta * 1000f)));
                if (_touchdownDownDuration > _requiredTouchdownDurationForWin)
                {
                    GD.Print("Won!");
                    EmitSignal(SignalName.ScoreChange, WinReward);
                    EmitSignal(SignalName.SimulationEnd);
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
    }

    private Dictionary<int, CollisionPolygon2D> CollisionObjectsById { get; set; } = new Dictionary<int, CollisionPolygon2D>();
    private bool _anyLegHasTouched = false;
    private int _numberOfTouchingLegs = 0;
    private bool _wasTouchingDownLastFrame = false;
    private TimeSpan _touchdownDownDuration;
    private readonly TimeSpan _requiredTouchdownDurationForWin = new(0, 0, 1);

    private void OnBodyShapeEntered(Rid _, Node _2, long _3, long local_shape_index)
	{
        CollisionPolygon2D localCollisionObject = GetLocalCollisionObject(local_shape_index);
		GD.Print($"Found leg {localCollisionObject.Name} Landing");

		if (localCollisionObject.Name.ToString().Contains("Leg"))
		{
			_numberOfTouchingLegs++;
			GD.Print(_numberOfTouchingLegs);
            if (false == _anyLegHasTouched)
            {
                _anyLegHasTouched = true;
                EmitSignal(SignalName.ScoreChange, FirstLegTouchReward);
            }
		}
        else if (localCollisionObject.Name.ToString().Contains("Body"))
        {
            GD.Print("CRASHED!");
            EmitSignal(SignalName.ScoreChange, CrashReward);
            EmitSignal(SignalName.SimulationEnd);
        }
    }

    private void OnBodyShapeExited(Rid _, Node _2, long _3, long local_shape_index)
    {
        CollisionPolygon2D localCollisionObject = GetLocalCollisionObject(local_shape_index);
        GD.Print($"Found leg {localCollisionObject.Name} Taking off");

        if (localCollisionObject.Name.ToString().Contains("Leg"))
        {
            _numberOfTouchingLegs--;
            GD.Print(_numberOfTouchingLegs);
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
        EmitSignal(SignalName.ScoreChange, ExceededMaxMissionLengthReward);
        GD.Print("Max Mission Lenght Exceeded");
        EmitSignal(SignalName.SimulationEnd);
    }
}



