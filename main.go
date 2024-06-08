package main

import (
	"fmt"
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
	initialInterval = time.Second / 5 // Initial interval for adding particles
	center          vector.Vector     // Center of the screen
)

const (
	Mass       = 2
	Shape      = "Circle"
	Radius     = 5     // Tiny particles
	Friction   = 1     // Friction coefficient
	Gravity    = 100   // Strength of gravity towards the center
	InitRadius = 300.0 // Initial radius of particle distribution
)
const (
	Viscosity     = 0.1 // Adjust the viscosity coefficient for desired effect
	HeatTransfer  = 0.1 // Rate of heat transfer between particles
	HeatThreshold = 0.5 // Threshold for heat-based repulsive force
)

var (
	particlesAdded = 0
	maxParticles   = 100 // Maximum number of particles to add
)

func update() error {
	// Add new particles until the maximum is reached
	if particlesAdded < maxParticles {
		select {
		case <-ticker.C:
			addParticle()
			addParticle()
			addParticle()
			particlesAdded++
			fmt.Println("Particles added: ", particlesAdded)
		default:
		}
	}
	for _, ball1 := range balls {
		for _, ball2 := range balls {
			if ball1 != ball2 {
				// Calculate distance and direction between particles
				distance := ball1.Position.Sub(ball2.Position)
				distanceMagnitude := distance.Magnitude()
				moveDirection := distance.Normalize()

				// Calculate repulsive force magnitude
				repulsiveForceMagnitude := 10 / (distanceMagnitude * distanceMagnitude) // Adjust the factor for desired repulsive effect

				// Apply repulsive force to velocities
				repulsiveForce := moveDirection.Scale(repulsiveForceMagnitude)
				ball1.Velocity = ball1.Velocity.Add(repulsiveForce.Scale(dt / ball1.Mass))

				// Transfer heat between particles
				heatTransfer := (ball1.Heat - ball2.Heat) * HeatTransfer
				ball1.Heat -= heatTransfer
				ball2.Heat += heatTransfer
			}
		}
	}

	for _, ball := range balls {
		// Apply gravity towards the center
		gravity := center.Sub(ball.Position).Normalize().Scale(Gravity)
		ApplyForcePVE(ball, gravity, dt)

		// Apply friction
		// ball.Velocity = ball.Velocity.Scale(Friction)

		// Apply viscosity
		viscosityForce := ball.Velocity.Scale(-Viscosity)
		ApplyForcePVE(ball, viscosityForce, dt)
	}

	// Resolve collisions and apply repulsive force to simulate thermal pressure
	for i := 0; i < len(balls); i++ {
		for j := i + 1; j < len(balls); j++ {
			if collision.CircleCollided(balls[i].RigidBody, balls[j].RigidBody) {
				resolveCollision(balls[i].RigidBody, balls[j].RigidBody)
				collision.BounceOnCollision(balls[i].RigidBody, balls[j].RigidBody, 0.9)
			}
		}
	}

	return nil
}

func resolveCollision(ball1, ball2 *rigidbody.RigidBody) {
	distance := ball1.Position.Sub(ball2.Position)
	distanceMagnitude := distance.Magnitude()
	minimumDistance := ball1.Radius + ball2.Radius

	if distanceMagnitude-(Radius/2) < minimumDistance {
		moveDirection := distance.Normalize()
		moveAmount := minimumDistance - distanceMagnitude

		// Move balls apart to resolve overlap
		moveVector1 := moveDirection.Scale(moveAmount / 2)
		moveVector2 := moveDirection.Scale(-moveAmount / 2)
		ball1.Position = ball1.Position.Add(moveVector1)
		ball2.Position = ball2.Position.Add(moveVector2)

		// Apply a repulsive force
		repulsiveForceMagnitude := moveAmount * 1 / (distanceMagnitude * distanceMagnitude) // Adjust the factor for desired repulsive effect
		repulsiveForce := moveDirection.Scale(repulsiveForceMagnitude)

		// Apply repulsive force to velocities
		ball1.Velocity = ball1.Velocity.Add(repulsiveForce.Scale(dt / ball1.Mass))
		ball2.Velocity = ball2.Velocity.Add(repulsiveForce.Scale(-dt / ball2.Mass))
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
	initializeBalls(10)

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
			Heat: 1.0, // Set initial heat value
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
		Heat: 1.0, // Set initial heat value
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
		// rb.Force = rb.Force.Add(force)

		body.Force = force
		acceleration := body.Force.Scale(1 / body.Mass)

		// Update velocity using acceleration and time step
		body.Velocity = body.Velocity.Add(acceleration.Scale(dt))

		// Update position using velocity and time step
		body.Position = body.Position.Add(body.Velocity.Scale(dt))
	}
}
