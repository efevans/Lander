using Godot;
using Godot.Collections;
using System;
using System.Collections.Generic;

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

    private List<StateActionPair> _stateActions = new List<StateActionPair>();

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
        Action action = new Action();

        // Use else-if here to restrict our action space to a size of 4
        if (Input.IsActionPressed("thrust_down"))
        {
            action.ThrustDown = true;
            var translatedForce = Transform.BasisXform(Vector2.Up * DownwardThrust);
            ApplyForce(translatedForce);
            ApplyReward(DownwardThrustReward);
        } 
        else if (Input.IsActionPressed("thrust_right"))
        {
            var translatedPosition = Transform.BasisXform(_rightThruster.Position);
            var translatedForce = Transform.BasisXform(Vector2.Left * SideThrust);
            ApplyForce(translatedForce, translatedPosition);
            ApplyReward(SideThrustReward);
        } 
        else if (Input.IsActionPressed("thrust_left"))
        {
            var translatedPosition = Transform.BasisXform(_leftThruster.Position);
            var translatedForce = Transform.BasisXform(Vector2.Right * SideThrust);
            ApplyForce(translatedForce, translatedPosition);
            ApplyReward(SideThrustReward);
        }
        else
        {
            action.Nothing = true;
        }

        if (LeftLegTouching && RightLegTouching)
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

        State state = new State(this);
        StateActionPair pair = new StateActionPair()
        {
            State = state,
            Action = action
        };
        _stateActions.Add(pair);
        GD.Print($"Size of state action pair buffer: {_stateActions.Count}");
    }

    private System.Collections.Generic.Dictionary<int, CollisionPolygon2D> CollisionObjectsById { get; set; } = new System.Collections.Generic.Dictionary<int, CollisionPolygon2D>();
    private bool _anyLegHasTouched = false;
    public bool LeftLegTouching { get; private set; } = false;
    public bool RightLegTouching { get; private set; } = false;
    private bool _wasTouchingDownLastFrame = false;
    private TimeSpan _touchdownDownDuration;
    private readonly TimeSpan _requiredTouchdownDurationForWin = new(0, 0, 1);
    private bool _simulationInProgress = false;

    private void OnBodyShapeEntered(Rid _, Node _2, long _3, long local_shape_index)
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
                LeftLegTouching = true;
            }
            else
            {
                RightLegTouching = true;
            }

        }
        else if (localCollisionObject.Name.ToString().Contains("Body"))
        {
            GD.Print("CRASHED!");
            ApplyReward(CrashReward);
            EndSimulation();
        }
    }

    private void OnBodyShapeExited(Rid _, Node _2, long _3, long local_shape_index)
    {
        CollisionPolygon2D localCollisionObject = GetLocalCollisionObject(local_shape_index);
        GD.Print($"Found leg {localCollisionObject.Name} Taking off");

        if (localCollisionObject.Name.ToString().Contains("Leg"))
        {
            if (localCollisionObject.Name.ToString().Contains("Left"))
            {
                LeftLegTouching = false;
            }
            else
            {
                RightLegTouching = false;
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
            EmitSignal(SignalName.SimulationEnd);
            _simulationInProgress = false;
        }
    }

    private void ApplyReward(float value)
    {
        if (_simulationInProgress)
        {
            EmitSignal(SignalName.ScoreChange, value);
        }
    }

    public class StateActionPair
    {
        public State State;
        public Action Action;
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
            LeftLegTouching = ship.LeftLegTouching;
            RightLegTouching = ship.RightLegTouching;
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

    public class Action
    {
        public bool Nothing;
        public bool ThrustDown;
        public bool ThrustLeft;
        public bool ThrustRight;
    }
}



