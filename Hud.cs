using Godot;
using System;

public partial class Hud : CanvasLayer
{
	private Label _scoreLabel;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
		_scoreLabel = GetNode<Label>("Reward");
	}

    public void UpdateScore(float score)
    {
        _scoreLabel.Text = score.ToString("n2");
    }
}
