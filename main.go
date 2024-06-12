package main

import (
	"image/color"
	"math"
	"math/rand"
	"time"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"
	"github.com/rudransh61/Physix-go/dynamics/collision"
	"github.com/rudransh61/Physix-go/pkg/rigidbody"
	"github.com/rudransh61/Physix-go/pkg/vector"
)

// PVEBody extends rigidbody.RigidBody with Heat field
type PVEBody struct {
	*rigidbody.RigidBody
	Heat float64 // Heat of the particle
}

var (
	balls           []*PVEBody // Using PVEBody instead of rigidbody.RigidBody
	dt              = 0.1
	ticker          *time.Ticker
	initialInterval = time.Second / 10 // Initial interval for adding particles
	center          vector.Vector      // Center of the screen
)

const (
	Mass       = 1
	Shape      = "Circle"
	Radius     = 2      // Tiny particles
	Friction   = 0.899  // Friction coefficient
	Gravity    = 50     // Strength of gravity towards the center
	InitRadius = 1000.0 // Initial radius of particle distribution
)
const (
	Viscosity     = 0.999 // Adjust the viscosity coefficient for desired effect
	HeatTransfer  = 0.1   // Rate of heat transfer between particles
	HeatThreshold = 0.5   // Threshold for heat-based repulsive force
)

var (
	particlesAdded = 0
	maxParticles   = 100000 // Maximum number of particles to add
)

func update() error {
	// Add new particles until the maximum is reached
	if particlesAdded < maxParticles {
		select {
		case <-ticker.C:
			addParticle()
			addParticle()
			addParticle()
			addParticle()
			addParticle()
			addParticle()
			addParticle()
			addParticle()
			addParticle()
			addParticle()
			addParticle()
			addParticle()
			addParticle()
			addParticle()
		default:
		}
	}
	for _, ball := range balls {
		// Apply gravity towards the center
		gravity := center.Sub(ball.Position).Normalize().Scale(Gravity)
		ApplyForcePVE(ball, gravity, dt)

		// Apply friction
		ball.Velocity = ball.Velocity.Scale(Friction)

		// Apply viscosity
		// ball.Velocity = ball.Velocity.Scale(-Viscosity)
		// ApplyForcePVE(ball, viscosityForce, dt)
	}

	// // Add gravitation
	// for i := 0; i < len(balls); i++ {
	// 	for j := i + 1; j < len(balls); j++ {
	// 		distance := vector.Distance(balls[i].Position, balls[j].Position)
	// 		gravitationalForce := balls[j].Position.Sub(balls[i].Position).Normalize().Scale(0.000001 * 0.000001 * 0.0000000000000000000000001 / (distance + 1500))
	// 		ApplyForcePVE(balls[i], gravitationalForce, dt)
	// 		ApplyForcePVE(balls[j], gravitationalForce.Scale(-1), dt)
	// 	}
	// }

	// Resolve collisions and apply repulsive force to simulate thermal pressure
	for i := 0; i < len(balls); i++ {
		for j := i + 1; j < len(balls); j++ {
			if collision.CircleCollided(balls[i].RigidBody, balls[j].RigidBody) {
				// collision.BounceOnCollision(balls[i].RigidBody, balls[j].RigidBody, 1.0)
				resolveCollision(balls[i].RigidBody, balls[j].RigidBody, balls)
			}
		}
	}

	return nil
}

func resolveCollision(ball1, ball2 *rigidbody.RigidBody, balls []*PVEBody) {
	distance := ball1.Position.Sub(ball2.Position)
	distanceMagnitude := distance.Magnitude()
	minimumDistance := ball1.Radius + ball2.Radius

	if distanceMagnitude < minimumDistance {
		moveDirection := distance.Normalize()
		overlap := minimumDistance - distanceMagnitude

		// Calculate the repulsive force magnitude based on the overlap
		mag := 10.0
		if len(balls) >= 1000 {
			mag = 350
		}
		repulsiveForceMagnitude := overlap * mag // Adjust this factor as needed for desired effect
		repulsiveForce := moveDirection.Scale(repulsiveForceMagnitude)

		// Apply the repulsive force to the velocities of the balls
		ball1.Velocity = ball1.Velocity.Add(repulsiveForce.Scale(dt / ball1.Mass))
		ball2.Velocity = ball2.Velocity.Add(repulsiveForce.Scale(-dt / ball2.Mass))

		// Adjust positions slightly to avoid sticking
		correctionFactor := 0.5 // Adjust this factor as needed for desired effect
		positionCorrection := moveDirection.Scale(correctionFactor * overlap)
		ball1.Position = ball1.Position.Add(positionCorrection)
		ball2.Position = ball2.Position.Sub(positionCorrection)
	}
}

func draw(screen *ebiten.Image) {
	for _, ball := range balls {
		// Determine the color based on heat
		colorValue := uint8(255 - ball.Heat - 10)

		// Set color based on heat
		color := color.RGBA{R: 0xff, G: colorValue, B: colorValue, A: 0xff}
		// fmt.Printf("color:", color)

		ebitenutil.DrawCircle(screen, ball.Position.X, ball.Position.Y, ball.Radius, color)
	}
}

func main() {
	ebiten.SetWindowSize(800, 800)
	ebiten.SetWindowTitle("Star Simulation")

	// Set the center of the screen
	center = vector.Vector{X: 400, Y: 400}

	ticker = time.NewTicker(initialInterval)

	// Initialize with a few particles
	initializeBalls(200)

	if err := ebiten.RunGame(&Game{}); err != nil {
		panic(err)
	}
}

func initializeBalls(n int) {
	balls = make([]*PVEBody, 0, n)
	for i := 0; i < n; i++ {
		// Generate particles in a circular pattern around the center
		angle := 2 * math.Pi * float64(i) / float64(n)
		radius := rand.Float64() * InitRadius
		x := center.X + radius*math.Cos(angle)
		y := center.Y + radius*math.Sin(angle)
		ball := &PVEBody{
			RigidBody: &rigidbody.RigidBody{
				Position:  vector.Vector{X: x, Y: y},
				Velocity:  vector.Vector{X: 0, Y: 0}, // No initial velocity
				Mass:      Mass,
				Shape:     Shape,
				Radius:    Radius,
				IsMovable: true,
			},
			Heat: 100.0, // Set initial heat value
		}
		balls = append(balls, ball)
	}
}

func addParticle() {
	// Add particles dynamically
	screenWidth, screenHeight := ebiten.WindowSize()
	x := rand.Float64() * float64(screenWidth)
	y := rand.Float64() * float64(screenHeight)
	ball := &PVEBody{
		RigidBody: &rigidbody.RigidBody{
			Position:  vector.Vector{X: x, Y: y},
			Velocity:  vector.Vector{X: 0, Y: 0}, // No initial velocity
			Mass:      Mass,
			Shape:     Shape,
			Radius:    Radius,
			IsMovable: true,
		},
		Heat: 100.0, // Set initial heat value
	}
	balls = append(balls, ball)
}

type Game struct{}

func (g *Game) Update() error {
	return update()
}

func (g *Game) Draw(screen *ebiten.Image) {
	draw(screen)
}

func (g *Game) Layout(outsideWidth, outsideHeight int) (screenWidth, screenHeight int) {
	return 800, 800
}

func ApplyForcePVE(body *PVEBody, force vector.Vector, dt float64) {
	if body.IsMovable {
		// Use Newton's second law: F = ma -> a = F/m
		body.Force = force
		acceleration := body.Force.Scale(1 / body.Mass)

		// Update velocity using acceleration and time step
		body.Velocity = body.Velocity.Add(acceleration.Scale(dt))

		// Update position using velocity and time step
		body.Position = body.Position.Add(body.Velocity.Scale(dt))

		body.Heat = body.Velocity.Scale(0.5).Magnitude()
	}
}
