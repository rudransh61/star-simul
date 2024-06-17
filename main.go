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
	"github.com/rudransh61/Physix-go/pkg/broadphase"
)

// PVEBody extends rigidbody.RigidBody with Heat field
type PVEBody struct {
	*rigidbody.RigidBody
	Heat  float64 // Heat of the particle
	Color color.RGBA
}

var (
	balls           []*PVEBody // Using PVEBody instead of rigidbody.RigidBody
	dt              = 0.1
	ticker          *time.Ticker
	initialInterval = time.Second / 5 // Initial interval for adding particles
	center          vector.Vector     // Center of the screen
	limit           = 10000

	// Broad-phase spatial hash
	spatialHash *broadphase.SpatialHash
)

const (
	Mass       = 1
	Shape      = "Circle"
	Radius     = 2     // Tiny particles
	Friction   = 0.899 // Friction coefficient
	Gravity    = 50    // Strength of gravity towards the center
	InitRadius = 1000.0 // Initial radius of particle distribution
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
			for i := 0; i < 14; i++ {
				addParticle()
			}
		default:
		}
	}

	// Clear spatial hash
	spatialHash.Clear()

	// Insert particles into the spatial hash
	for _, ball := range balls {
		spatialHash.Add(ball, ball.Position)
	}

	// Apply forces and update particles
	for _, ball := range balls {
		// Apply gravity towards the center
		gravity := center.Sub(ball.Position).Normalize().Scale(Gravity)
		ApplyForcePVE(ball, gravity, dt)

		// Apply friction
		ball.Velocity = ball.Velocity.Scale(Friction)
	}

	// Broad-phase collision detection and response
	for i := 0; i < len(balls); i++ {
		ball := balls[i]

		// Query nearby objects using spatial hash
		nearbyObjects := spatialHash.Query(ball.Position)
		for _, obj := range nearbyObjects {
			other, ok := obj.(*PVEBody)
			if !ok || other == ball {
				continue
			}

			// Check collision and resolve
			if collision.CircleCollided(ball.RigidBody, other.RigidBody) {
				resolveCollision(ball.RigidBody, other.RigidBody, balls)
			}
		}
	}

	return nil
}

func draw(screen *ebiten.Image) {
	for _, ball := range balls {
		// Draw the particle with its current color
		ebitenutil.DrawCircle(screen, ball.Position.X, ball.Position.Y, ball.Radius, ball.Color)
	}
}


func main() {
	ebiten.SetWindowSize(800, 800)
	ebiten.SetWindowTitle("Star Simulation")

	// Set the center of the screen
	center = vector.Vector{X: 400, Y: 400}

	ticker = time.NewTicker(initialInterval)

	// Initialize spatial hash with appropriate cell size
	cellSize := 2.0 * Radius // Adjust cell size based on particle size
	screenWidth, screenHeight := ebiten.WindowSize()
	spatialHash = broadphase.NewSpatialHash(cellSize, float64(screenWidth), float64(screenHeight))

	// Initialize with a few particles
	initializeBalls(100)

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
			Color: HeatToColor(0), // Set initial color based on initial heat
			Heat:  0,              // Set initial heat value
		}
		balls = append(balls, ball)
	}
}


func addParticle() {
	// Add particles dynamically
	screenWidth, screenHeight := ebiten.WindowSize()
	x := rand.Float64() * float64(screenWidth)
	y := rand.Float64() * float64(screenHeight)
	colorValue := uint8(rand.Int())
	colorValue1 := uint8(rand.Int())
	colorValue2 := uint8(rand.Int())
	ball := &PVEBody{
		RigidBody: &rigidbody.RigidBody{
			Position:  vector.Vector{X: x, Y: y},
			Velocity:  vector.Vector{X: 0, Y: 0}, // No initial velocity
			Mass:      Mass,
			Shape:     Shape,
			Radius:    Radius,
			IsMovable: true,
		},
		Color: color.RGBA{R: colorValue1, G: colorValue2, B: colorValue, A: 0xff},
		Heat:  100.0, // Set initial heat value
	}
	balls = append(balls, ball)
	particlesAdded++
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

		// Update heat based on velocity magnitude
		body.Heat = body.Velocity.Magnitude()

		// Update color based on heat
		body.Color = HeatToColor(body.Heat)
	}
}


func resolveCollision(ball1, ball2 *rigidbody.RigidBody, balls []*PVEBody) {
	distance := ball1.Position.Sub(ball2.Position)
	distanceMagnitude := distance.Magnitude()
	minimumDistance := ball1.Radius + ball2.Radius

	if distanceMagnitude < minimumDistance {
		moveDirection := distance.Normalize()
		overlap := (minimumDistance - distanceMagnitude) * 5

		// Calculate the repulsive force magnitude based on the overlap
		mag := 10.0
		repulsiveForceMagnitude := overlap * mag // Adjust this factor as needed for desired effect
		repulsiveForce := moveDirection.Scale(repulsiveForceMagnitude)

		// Apply the repulsive force to the velocities of the balls
		ball1.Velocity = ball1.Velocity.Add(repulsiveForce.Scale(dt / ball1.Mass).Scale(0.9))
		ball2.Velocity = ball2.Velocity.Add(repulsiveForce.Scale(-dt / ball2.Mass).Scale(0.9))

		// Adjust positions slightly to avoid sticking
		correctionFactor := 0.5 // Adjust this factor as needed for desired effect
		positionCorrection := moveDirection.Scale(correctionFactor * overlap * 5)
		ball1.Force = ball1.Force.Add(positionCorrection)
		ball2.Force = ball2.Force.Sub(positionCorrection)
	}
}

func HeatToColor(heat float64) color.RGBA {
	// Example color mapping: blue for low heat, red for medium heat, white for high heat
	// Heat range: 0 - 1000
	if heat < 0 {
		heat = 0
	}
	if heat > 1000 {
		heat = 1000
	}

	r := uint8(0)
	g := uint8(0)
	b := uint8(0)

	if heat < 500 {
		b = 255
		g = uint8(255 * (heat / 500))
	} else {
		r = 255
		g = uint8(255 * ((1000 - heat) / 500))
	}

	return color.RGBA{R: r, G: g, B: b, A: 255}
}
