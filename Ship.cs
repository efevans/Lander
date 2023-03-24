using Godot;
using Godot.Collections;
using System;

public partial class Ship : RigidBody2D
{
    [Export]
    public int DownwardThrust = 300;
    [Export]
    public int SideThrust = 100;

    private Sprite2D _leftThruster;
    private Sprite2D _rightThruster;

    private Dictionary<int, CollisionPolygon2D> CollisionObjectsById { get; set; } = new Dictionary<int, CollisionPolygon2D>();
	private int _numberOfTouchingLegs = 0;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
        _leftThruster = GetNode<Sprite2D>("ThrusterLeftSprite");
        _rightThruster = GetNode<Sprite2D>("ThrusterRightSprite");
    }

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
        var angle = Rotation;

        if (Input.IsActionPressed("thrust_down"))
        {
            var translatedPosition = Transform.BasisXform(_rightThruster.Position);
            var translatedForce = Transform.BasisXform(Vector2.Up * DownwardThrust);
            ApplyForce(translatedForce, translatedPosition);
        }

        if (Input.IsActionPressed("thrust_right"))
        {
            var translatedPosition = Transform.BasisXform(_rightThruster.Position);
            var translatedForce = Transform.BasisXform(Vector2.Left * SideThrust);
            ApplyForce(translatedForce, translatedPosition);
        }

        if (Input.IsActionPressed("thrust_left"))
        {
            var translatedPosition = Transform.BasisXform(_leftThruster.Position);
            var translatedForce = Transform.BasisXform(Vector2.Right * SideThrust);
            ApplyForce(translatedForce, translatedPosition);
        }
    }

	private void OnBodyShapeEntered(Rid _, Node _2, long _3, long local_shape_index)
	{
        CollisionPolygon2D localCollisionObject = GetLocalCollisionObject(local_shape_index);
		GD.Print($"Found leg {localCollisionObject.Name} Landing");

		if (localCollisionObject.Name.ToString().Contains("Leg"))
		{
			_numberOfTouchingLegs++;
			GD.Print(_numberOfTouchingLegs);
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
}



