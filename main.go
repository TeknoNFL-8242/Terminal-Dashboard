package main

import (
	"fmt"
	"math"
	"strconv"
	"strings"
	"time"

	tea "github.com/charmbracelet/bubbletea"
	"github.com/veandco/go-sdl2/sdl"
)

// --- Uygulama Modları ---
type ViewMode int

const (
	ModeDashboard ViewMode = iota
	ModeGraph
	ModeHelp
	ModeSettings
	ModeController
	ModeShoot
)

const version string = "1.4/Final-Sim"

// ============================================================================
// 2026 REBUILT (Manual tabanlı saha/HUB parametreleri)
// ============================================================================

// Saha
const (
	FieldLength = 16.54 // m
	FieldWidth  = 8.21  // m
	Gravity     = 9.81  // m/s^2
	MidLine     = 8.27  // m (FieldLength/2)
)

// 2026 HUB ve ALLIANCE ZONE (kaba midline yerine “zone-depth”)
// Not: Bu değerler “manual tabanlı” (senin verdiğin) — gerektiğinde güncellersin.
const (
	HubOpeningHeight   = 1.83 // m (72 in) - opening height referansı
	HubFromWallX       = 4.03 // m (158.6 in) - HUB center x offset from alliance wall
	AllianceZoneDepth  = 4.03 // m - zone depth from alliance wall
	AllianceZoneLength = 8.07 // m - Y band length (almost full field width)
)

// HUB Y: field center
const HubCenterY = FieldWidth / 2.0

// ============================================================================
// Flywheel / Ball / Aero
// ============================================================================

const (
	WheelRadiusM = 0.0508 // 4 inch wheel radius

	BallMassKg    = 0.210
	BallDiameterM = 5.91 * 0.0254
	BallRadiusM   = BallDiameterM / 2.0

	AirDensity = 1.225
	DragCd     = 0.47 // sphere baseline
)

const (
	MechanicalEfficiency = 0.85
)

const (
	SpinFraction = 0.55
	MaxSimTime   = 3.0
	Dt           = 0.002
)

const (
	MaxReasonableRPM = 9000.0
)

// ============================================================================

type Alliance int

const (
	BlueAlliance Alliance = iota
	RedAlliance
)

type JoystickMsg struct {
	X, Y, Z, Slider float64
	Buttons         [12]bool
	POV             int
	Connected       bool
}

type RefreshMsg struct{}

// --- Veri Modeli ---
type model struct {
	// Inputs
	x, y, z, slider float64
	button1         bool
	joyOK           bool
	buttons         [12]bool
	pov             int

	manualOverride bool
	lastJoyX       float64
	lastJoyY       float64

	// Drive params
	k, a float64

	// Shoot params
	robotX        float64
	robotY        float64
	shooterHeight float64
	shooterAngle  float64
	alliance      Alliance

	// Outputs
	left, right float64

	// UI
	mode    ViewMode
	input   string
	lastMsg string
	width   int
	height  int
}

func initialModel() model {
	return model{
		k:             2.3,
		a:             2.9,
		mode:          ModeDashboard,
		robotX:        2.0,
		robotY:        4.1,
		shooterHeight: 0.51,
		shooterAngle:  25.0,
		alliance:      BlueAlliance,
	}
}

// --- Drive math ---
func calculateLogic(m *model) {
	y_ax := m.y
	x_ax := m.x
	z_ax := m.z

	if m.button1 {
		m.left = y_ax
		m.right = y_ax
		return
	}

	if math.Abs(x_ax) < 0.05 && math.Abs(y_ax) < 0.05 && math.Abs(z_ax) > 0.05 {
		m.left = z_ax
		m.right = -z_ax
		return
	}

	if math.Abs(x_ax) > 0.05 && math.Abs(y_ax) > 0.05 {
		xAbs := math.Abs(x_ax)
		yAbs := math.Abs(y_ax)

		logK := math.Log(1.0 + m.k)
		f := math.Log(1.0+m.k*xAbs) / logK
		g := math.Pow(1.0-yAbs, m.a)

		if x_ax > 0 && y_ax > 0 {
			m.left = yAbs + yAbs*f*g
			m.right = yAbs * (1.0 - f)
		} else if x_ax < 0 && y_ax > 0 {
			m.left = yAbs * (1.0 - f)
			m.right = yAbs + yAbs*f*g
		} else if x_ax > 0 && y_ax < 0 {
			m.left = -yAbs - yAbs*f*g
			m.right = -yAbs * (1.0 - f)
		} else if x_ax < 0 && y_ax < 0 {
			m.left = -yAbs * (1.0 - f)
			m.right = -yAbs - yAbs*f*g
		}
		return
	}

	if math.Abs(y_ax) > 0.05 {
		m.left = y_ax
		m.right = y_ax
		return
	}

	m.left = 0
	m.right = 0
}

func f_val(x, k float64) float64 {
	return math.Log(1.0+k*math.Abs(x)) / math.Log(1.0+k)
}

func g_val(y, a float64) float64 {
	return math.Pow(1.0-math.Abs(y), a)
}

// ============================================================================
// SHOOT PHYSICS (Drag + Magnus + Binary Search) — FIXED: distance-based solve
// ============================================================================

type PhysicsResult struct {
	IsPossible     bool
	TargetVelocity float64
	TargetRPM      float64
	MaxHeight      float64
	FlightTime     float64
	Distance       float64
	Reason         string
	Legal          bool
	LegalReason    string
}

// zone status: legal flag only (shot math still computed)
func allianceZoneStatus(m *model) (bool, string) {
	if m.robotX < 0 || m.robotX > FieldLength || m.robotY < 0 || m.robotY > FieldWidth {
		return false, "OUT OF FIELD"
	}

	zoneYMin := (FieldWidth - AllianceZoneLength) / 2.0
	zoneYMax := zoneYMin + AllianceZoneLength
	if m.robotY < zoneYMin || m.robotY > zoneYMax {
		return false, "Y OUTSIDE ALLIANCE ZONE BAND"
	}

	if m.alliance == BlueAlliance {
		if m.robotX > AllianceZoneDepth {
			return false, "NOT IN BLUE ALLIANCE ZONE (X DEPTH)"
		}
	} else {
		if m.robotX < (FieldLength - AllianceZoneDepth) {
			return false, "NOT IN RED ALLIANCE ZONE (X DEPTH)"
		}
	}

	return true, "OK"
}

func hubTarget(alliance Alliance) (tx, ty float64) {
	ty = HubCenterY
	if alliance == BlueAlliance {
		tx = HubFromWallX
	} else {
		tx = FieldLength - HubFromWallX
	}
	return
}

func velocityToRPM(v0 float64) float64 {
	wheelV := v0 / MechanicalEfficiency
	radPerSec := wheelV / WheelRadiusM
	return radPerSec * 60.0 / (2.0 * math.Pi)
}

// Cl(S) ≈ 1.2*S/(1+S) (stable)
func liftCoeffFromSpin(S float64) float64 {
	if S <= 0 {
		return 0
	}
	return 1.2 * S / (1.0 + S)
}

// Simulate until x reaches targetDist, then interpolate y at that x.
// Returns:
//
//	yErr = y_at_targetDist - targetHeight  (positive => above, negative => below)
//	maxHeight
//	tAtDist
//	reachedDist: whether we got to targetDist before sim ended
func simulateToDistanceDragMagnus(
	v0 float64,
	thetaRad float64,
	shooterH float64,
	targetDist float64,
	targetHeight float64,
) (yErr, maxH, tAtDist float64, reachedDist bool) {

	area := math.Pi * BallRadiusM * BallRadiusM
	dragK := 0.5 * AirDensity * DragCd * area / BallMassKg

	// spin model (simple coupling)
	wheelSpeed := v0 / MechanicalEfficiency
	omegaBall := SpinFraction * (wheelSpeed / BallRadiusM)

	x, y := 0.0, shooterH
	vx := v0 * math.Cos(thetaRad)
	vy := v0 * math.Sin(thetaRad)

	maxH = y
	t := 0.0

	prevX, prevY := x, y

	for t < MaxSimTime && y > 0.0 {
		speed := math.Hypot(vx, vy)
		if speed < 1e-6 {
			break
		}

		// drag
		axDrag := -dragK * speed * vx
		ayDrag := -dragK * speed * vy

		// lift (up)
		S := (omegaBall * BallRadiusM) / speed
		cl := liftCoeffFromSpin(S)
		liftK := 0.5 * AirDensity * cl * area / BallMassKg
		ayLift := liftK * speed * speed

		ax := axDrag
		ay := ayDrag + ayLift - Gravity

		// semi-implicit euler
		vx += ax * Dt
		vy += ay * Dt

		prevX, prevY = x, y
		x += vx * Dt
		y += vy * Dt
		t += Dt

		if y > maxH {
			maxH = y
		}

		// reached target distance? interpolate y at exact x=targetDist
		if prevX < targetDist && x >= targetDist {
			den := x - prevX
			if math.Abs(den) < 1e-9 {
				den = 1e-9
			}
			alpha := (targetDist - prevX) / den
			if alpha < 0 {
				alpha = 0
			} else if alpha > 1 {
				alpha = 1
			}
			yAtDist := prevY + alpha*(y-prevY)
			tAtDist = (t - Dt) + alpha*Dt
			return yAtDist - targetHeight, maxH, tAtDist, true
		}
	}

	// did not reach the target distance
	return (y - targetHeight), maxH, t, false
}

func calculateShootPhysics(m *model) PhysicsResult {
	legal, legalReason := allianceZoneStatus(m)

	tx, ty := hubTarget(m.alliance)
	dx := tx - m.robotX
	dy := ty - m.robotY
	dist := math.Hypot(dx, dy)

	thetaRad := m.shooterAngle * (math.Pi / 180.0)

	// Binary search on v0 to make y_at_dist ~= HubOpeningHeight
	vLow := 3.0
	vHigh := 55.0 // allow higher so solver can still return a value

	tolY := 0.05 // 5cm height tolerance at x=dist

	bestV := -1.0
	bestAbsErr := 1e18
	bestMaxH := 0.0
	bestT := 0.0
	bestReached := false
	bestYErr := 0.0

	for i := 0; i < 32; i++ {
		vMid := 0.5 * (vLow + vHigh)

		yErr, maxH, tAtDist, reachedDist := simulateToDistanceDragMagnus(
			vMid, thetaRad, m.shooterHeight, dist, HubOpeningHeight,
		)

		if reachedDist {
			bestReached = true
			absErr := math.Abs(yErr)
			if absErr < bestAbsErr {
				bestAbsErr = absErr
				bestV = vMid
				bestMaxH = maxH
				bestT = tAtDist
				bestYErr = yErr
			}

			if absErr < tolY {
				bestV = vMid
				bestMaxH = maxH
				bestT = tAtDist
				bestYErr = yErr
				break
			}

			// yErr > 0 => too high => reduce v
			// yErr < 0 => too low  => increase v
			if yErr > 0 {
				vHigh = vMid
			} else {
				vLow = vMid
			}
		} else {
			// couldn't even reach distance => need more velocity
			vLow = vMid
		}
	}

	// Always output something meaningful (no more 0/0)
	if !bestReached {
		yErrHi, maxHHi, tHi, _ := simulateToDistanceDragMagnus(
			vHigh, thetaRad, m.shooterHeight, dist, HubOpeningHeight,
		)
		return PhysicsResult{
			IsPossible:     false,
			TargetVelocity: vHigh,
			TargetRPM:      velocityToRPM(vHigh),
			MaxHeight:      maxHHi,
			FlightTime:     tHi,
			Distance:       dist,
			Reason:         fmt.Sprintf("CANNOT_REACH_DISTANCE (yErr@vHigh=%.2fm)", yErrHi),
			Legal:          legal,
			LegalReason:    legalReason,
		}
	}

	targetRPM := velocityToRPM(bestV)

	reason := "READY"
	isPossible := true

	if bestAbsErr >= tolY {
		reason = fmt.Sprintf("NO_CONVERGENCE (BEST yErr=%.2fm)", bestYErr)
		// yine de değerleri basıyoruz; kullanıcı “hedefe yakın” görsün
	}

	if targetRPM > MaxReasonableRPM {
		reason = reason + " | RPM_TOO_HIGH?"
	}

	return PhysicsResult{
		IsPossible:     isPossible,
		TargetVelocity: bestV,
		TargetRPM:      targetRPM,
		MaxHeight:      bestMaxH,
		FlightTime:     bestT,
		Distance:       dist,
		Reason:         reason,
		Legal:          legal,
		LegalReason:    legalReason,
	}
}

// ============================================================================
// Bubble Tea
// ============================================================================

func (m model) Init() tea.Cmd {
	return tea.Batch(tick(), tea.EnterAltScreen)
}

func tick() tea.Cmd {
	return tea.Tick(30*time.Millisecond, func(t time.Time) tea.Msg { return t })
}

func (m model) Update(msg tea.Msg) (tea.Model, tea.Cmd) {
	switch msg := msg.(type) {
	case tea.WindowSizeMsg:
		m.width = msg.Width
		m.height = msg.Height

	case JoystickMsg:
		m.joyOK = msg.Connected

		if msg.Connected {
			joyMoved := math.Abs(msg.X-m.lastJoyX) > 0.02 || math.Abs(msg.Y-m.lastJoyY) > 0.02
			if joyMoved {
				m.manualOverride = false
			}
			if !m.manualOverride {
				m.x = msg.X
				m.y = msg.Y
			}
			m.z = msg.Z
			m.slider = msg.Slider
			m.lastJoyX = msg.X
			m.lastJoyY = msg.Y
		}

		m.buttons = msg.Buttons
		m.pov = msg.POV
		m.button1 = msg.Buttons[0]

		calculateLogic(&m)
		return m, nil

	case RefreshMsg:
		return m, nil

	case time.Time:
		// tick redraw
		calculateLogic(&m)
		return m, tick()

	case tea.KeyMsg:
		switch msg.String() {
		case "ctrl+c":
			return m, tea.Quit
		case "backspace":
			if len(m.input) > 0 {
				m.input = m.input[:len(m.input)-1]
			}
		case "enter":
			return m.processInput()
		default:
			// keep it simple
			if len(msg.String()) == 1 {
				m.input += msg.String()
			}
		}
	}
	return m, nil
}

func (m model) processInput() (tea.Model, tea.Cmd) {
	cmd := strings.TrimSpace(strings.ToLower(m.input))
	m.input = ""

	switch cmd {
	case "help":
		m.mode = ModeHelp
		m.lastMsg = "Help opened."
		return m, nil
	case "graph":
		m.mode = ModeGraph
		m.lastMsg = "Switched to Graph Mode."
		return m, nil
	case "dash", "dashboard":
		m.mode = ModeDashboard
		m.lastMsg = "Switched to Dashboard."
		return m, nil
	case "controller", "joy", "joystick":
		m.mode = ModeController
		m.lastMsg = "Controller Diagnostic Mode active."
		return m, nil
	case "shoot", "sim", "twin":
		m.mode = ModeShoot
		m.lastMsg = "Switched to Shoot/Digital Twin Mode."
		return m, nil
	case "red":
		m.alliance = RedAlliance
		m.lastMsg = "Alliance set to RED."
		return m, nil
	case "blue":
		m.alliance = BlueAlliance
		m.lastMsg = "Alliance set to BLUE."
		return m, nil
	case "refresh":
		m.lastMsg = "Joystick status refreshed."
		return m, func() tea.Msg { return RefreshMsg{} }
	case "settings":
		m.mode = ModeSettings
		return m, nil
	case "return":
		m.mode = ModeDashboard
		m.lastMsg = "Returning to Dashboard."
		return m, nil
	case "quit", "q", "exit":
		return m, tea.Quit
	}

	if strings.Contains(cmd, "=") {
		parts := strings.Split(cmd, "=")
		if len(parts) == 2 {
			val, err := strconv.ParseFloat(parts[1], 64)
			if err == nil {
				switch parts[0] {
				case "k":
					m.k = val
				case "a":
					m.a = val
				case "x":
					m.x = val
					m.manualOverride = true
				case "y":
					m.y = val
					m.manualOverride = true
				case "z":
					m.z = val

				case "xloc":
					if val >= 0 && val <= FieldLength {
						m.robotX = val
						m.lastMsg = fmt.Sprintf("xloc set to %.2f", val)
					} else {
						m.lastMsg = "Error: X out of bounds!"
					}
				case "yloc":
					if val >= 0 && val <= FieldWidth {
						m.robotY = val
						m.lastMsg = fmt.Sprintf("yloc set to %.2f", val)
					} else {
						m.lastMsg = "Error: Y out of bounds!"
					}
				case "sh":
					m.shooterHeight = val
					m.lastMsg = fmt.Sprintf("sh set to %.2f", val)
				case "sa":
					m.shooterAngle = val
					m.lastMsg = fmt.Sprintf("sa set to %.2f", val)
				default:
					m.lastMsg = "Unknown variable."
				}

				// force immediate visible update
				return m, func() tea.Msg { return RefreshMsg{} }
			}
		}
	}

	m.lastMsg = "Unknown command."
	return m, nil
}

// ============================================================================
// Views
// ============================================================================

func (m model) View() string {
	joyStatus := "UNAVALIBLE ❌"
	if m.joyOK {
		joyStatus = "CONNECTED ✅"
	}

	header := fmt.Sprintf("\n FRC TERMINAL DASHBOARD | v%s | Mode: %s | Joystick: %s\n",
		version, modeName(m.mode), joyStatus)

	var content string
	switch m.mode {
	case ModeDashboard:
		content = m.viewDashboard()
	case ModeGraph:
		content = m.viewGraph()
	case ModeHelp:
		content = m.viewHelp()
	case ModeSettings:
		content = m.viewSettings()
	case ModeController:
		content = m.viewController()
	case ModeShoot:
		content = m.viewShoot()
	}

	footer := fmt.Sprintf("\n Command > %s_\n %s", m.input, m.lastMsg)
	return header + content + footer
}

func modeName(m ViewMode) string {
	switch m {
	case ModeDashboard:
		return "DASHBOARD"
	case ModeGraph:
		return "GRAPH ANALYZER"
	case ModeHelp:
		return "HELP"
	case ModeSettings:
		return "SETTINGS"
	case ModeController:
		return "CONTROLLER DIAGNOSTIC"
	case ModeShoot:
		return "SHOOT SIMULATION (DIGITAL TWIN)"
	default:
		return "?"
	}
}

// --- SHOOT VIEW ---
func (m model) viewShoot() string {
	res := calculateShootPhysics(&m)
	tx, ty := hubTarget(m.alliance)

	const mapW = 64
	const mapH = 14

	var sb strings.Builder
	sb.WriteString("\n  SAHA HARİTASI (Kuş Bakışı) - [BLUE 0,0] -> [RED 16.54,0]\n")
	sb.WriteString("  ┌" + strings.Repeat("─", mapW) + "┐\n")

	for y := mapH - 1; y >= 0; y-- {
		sb.WriteString("  │")
		for x := 0; x < mapW; x++ {
			realX := (float64(x) / float64(mapW)) * FieldLength
			realY := (float64(y) / float64(mapH)) * FieldWidth

			char := " "

			// HUB
			isHub := math.Abs(realX-tx) < 0.6 && math.Abs(realY-ty) < 0.8
			if isHub {
				if m.alliance == BlueAlliance {
					char = "\033[34mH\033[0m"
				} else {
					char = "\033[31mH\033[0m"
				}
			}

			// Robot
			if !isHub && math.Abs(realX-m.robotX) < 0.3 && math.Abs(realY-m.robotY) < 0.3 {
				char = "\033[33m▣\033[0m"
			} else if !isHub {
				// Midline
				if math.Abs(realX-MidLine) < 0.15 {
					char = "┆"
				}
				// Alliance zone depth marker
				if m.alliance == BlueAlliance && math.Abs(realX-AllianceZoneDepth) < 0.10 {
					char = "╎"
				}
				if m.alliance == RedAlliance && math.Abs(realX-(FieldLength-AllianceZoneDepth)) < 0.10 {
					char = "╎"
				}
			}

			sb.WriteString(char)
		}
		sb.WriteString("│\n")
	}
	sb.WriteString("  └" + strings.Repeat("─", mapW) + "┘\n")

	allianceStr := "\033[34mBLUE\033[0m"
	if m.alliance == RedAlliance {
		allianceStr = "\033[31mRED\033[0m"
	}

	legalStr := "\033[32mLEGAL\033[0m"
	if !res.Legal {
		legalStr = "\033[31mILLEGAL\033[0m"
	}

	simStatus := "\033[31mIMPOSSIBLE SHOT ❌\033[0m"
	if res.IsPossible {
		simStatus = "\033[32mSHOOTABLE ✅\033[0m"
	}

	analysis := fmt.Sprintf(`
  ROBOT STATE:
  Alliance: %s  | Pos: X=%.2fm, Y=%.2fm   | Zone: %s (%s)
  Shooter : Height=%.2fm, Angle=%.1f°
  HUB     : X=%.2fm, Y=%.2fm, OpeningHeight=%.2fm

  SIMULATION RESULT: %s (%s)
  Distance to HUB   : %.2f m
  Target Velocity   : %.2f m/s
  Target RPM        : %.0f RPM
  Max Apex Height   : %.2f m
  Time of Flight    : %.2f s
`, allianceStr, m.robotX, m.robotY, legalStr, res.LegalReason,
		m.shooterHeight, m.shooterAngle,
		tx, ty, HubOpeningHeight,
		simStatus, res.Reason,
		res.Distance, res.TargetVelocity, res.TargetRPM, res.MaxHeight, res.FlightTime)

	return sb.String() + analysis
}

func (m model) viewDashboard() string {
	statusText := "DISABLE❌"
	if m.joyOK {
		statusText = "ENABLE ✅"
	}

	return fmt.Sprintf(`
 ┌── Joystick ─────────────────────────────┐
 │ X: %-6.2f  Y: %-6.2f  Z: %-6.2f         │
 │ Trigger: %-5v /Controller: %-11s│
 └── Parameters ───────────────────────────┘
 │ k (Log): %-6.2f   a (Pow): %-6.2f       │
 └─────────────────────────────────────────┘
 ┌── Motors ───────────────────────────────┐
 │ Left : %s %+.2f     │
 │ Right: %s %+.2f     │
 └─────────────────────────────────────────┘
`, m.x, m.y, m.z, m.button1, statusText, m.k, m.a,
		makeBar(m.left), m.left,
		makeBar(m.right), m.right)
}

func (m model) viewGraph() string {
	var sb strings.Builder
	sb.WriteString("\n  ANALİZ GRAFİĞİ (0..1 EKSENİ)\n")
	sb.WriteString("  1.0 +------------------------------+\n")

	for i := 10; i >= 0; i-- {
		yLine := float64(i) / 10.0
		sb.WriteString(fmt.Sprintf("  %3.1f |", yLine))

		for j := 0; j <= 30; j++ {
			xInput := float64(j) / 30.0

			f := f_val(xInput, m.k)
			g := g_val(xInput, m.a)

			char := " "
			isF := math.Abs(f-yLine) < 0.05
			isG := math.Abs(g-yLine) < 0.05

			if isF && isG {
				char = "@"
			} else if isF {
				char = "*"
			} else if isG {
				char = "#"
			}
			sb.WriteString(char)
		}
		sb.WriteString("|\n")
	}
	sb.WriteString("  0.0 +------------------------------+\n")
	sb.WriteString("      0.0   (* = f(x) | # = g(y))    1.0\n")

	staticText := `
  FORMÜLLER:
  f(x) = ln(1 + k*|x|) / ln(1 + k)    [Dönüş Oranı]
  g(y) = (1 - |y|)^a                  [Boost Sönümü]
`
	xAbs := math.Abs(m.x)
	yAbs := math.Abs(m.y)
	fVal := f_val(m.x, m.k)
	gVal := g_val(m.y, m.a)

	dynamicText := fmt.Sprintf(`
  HESAPLAMA (Anlık):
  f(%.2f) = ln(1 + %.1f*%.2f) / ln(1 + %.1f) = %.3f
  g(%.2f) = (1 - %.2f)^%.1f               = %.3f
`, xAbs, m.k, xAbs, m.k, fVal, yAbs, yAbs, m.a, gVal)

	robotBox := fmt.Sprintf(`
    Left: %+.2f              Right: %+.2f
    +---------+             +---------+
    |   [M]   |=============|   [M]   |
    |         |    ROBOT    |         |
    |    <    |      ^      |    >    |
    +---------+             +---------+
`, m.left, m.right)

	return sb.String() + staticText + dynamicText + robotBox
}

func (m model) viewHelp() string {
	return `
  ┌──────────────── KOMUT LİSTESİ ────────────────┐
  │  Modlar:                                      │
  │    shoot        -> Atış Simülasyonu (Twin)    │
  │    graph        -> Grafik ve Analiz ekranı    │
  │    dash         -> Basit Dashboard ekranı     │
  │    help         -> Yardım                     │
  │    controller   -> Controller diagnostic      │
  │    settings     -> Ayarlar                    │
  │                                               │
  │  Shoot Mode Ayarları:                         │
  │    red / blue   -> Alliance seçimi            │
  │    xloc=2.5     -> Robot X (m)                │
  │    yloc=4.0     -> Robot Y (m)                │
  │    sh=0.51      -> Shooter height (m)         │
  │    sa=25        -> Shooter angle (deg)        │
  │                                               │
  │  Araçlar:                                     │
  │    refresh      -> Joystick durumunu yenile   │
  │    return       -> Dashboard'a dön            │
  │    quit         -> Çıkış                      │
  └───────────────────────────────────────────────┘
`
}

func (m model) viewSettings() string {
	return `
 ┌──────────────── SETTINGS MODE ────────────────┐
 │  Komutlar:                                    │
 │  > k=value   : k değerini değiştirir          │
 │  > a=value   : a değerini değiştirir          │
 │  > return    : Dashboard'a geri dön           │
 └───────────────────────────────────────────────┘
`
}

func makeBar(val float64) string {
	width := 20
	half := width / 2
	absVal := math.Abs(val)

	if val > 1 {
		val = 1
	}
	if val < -1 {
		val = -1
	}

	pos := int(val * float64(half))

	var sb strings.Builder
	sb.WriteString("[")

	for i := 0; i < half; i++ {
		charPos := i - half
		if pos < 0 && charPos >= pos {
			colorCode := 124
			if absVal > 0.7 {
				colorCode = 196
			} else if absVal > 0.3 {
				colorCode = 160
			}
			sb.WriteString(fmt.Sprintf("\033[38;5;%dm█\033[0m", colorCode))
		} else {
			sb.WriteString("░")
		}
	}

	sb.WriteString("|")

	for i := 0; i < half; i++ {
		if pos > 0 && i < pos {
			colorCode := 28
			if absVal > 0.7 {
				colorCode = 46
			} else if absVal > 0.3 {
				colorCode = 34
			}
			sb.WriteString(fmt.Sprintf("\033[38;5;%dm█\033[0m", colorCode))
		} else {
			sb.WriteString("░")
		}
	}

	sb.WriteString("]")
	return sb.String()
}

func (m model) viewController() string {
	if !m.joyOK {
		return `
 ┌── LOGITECH EXTREME 3D PRO DIAGNOSTIC ──────────────────────────────────┐
 │              ⚠️  HATA: CONTROLLER BAĞLI DEĞİL!  ⚠️                     │
 │    Lütfen cihazın USB bağlantısını kontrol edin ve 'refresh' yazın.    │
 └────────────────────────────────────────────────────────────────────────┘
`
	}

	var btnRows []string
	for i := 0; i < 3; i++ {
		row := ""
		for j := 1; j <= 4; j++ {
			idx := (i * 4) + j - 1
			status := "○"
			if m.buttons[idx] {
				status = "●"
			}
			row += fmt.Sprintf("%2d:%s  ", idx+1, status)
		}
		btnRows = append(btnRows, row)
	}

	povDisplay := getPOVDisplaySimple(m.pov)

	return fmt.Sprintf(`
 ┌── LOGITECH EXTREME 3D PRO DIAGNOSTIC ──────────────────────────────────┐
 │  STICKS (Analog)                          BUTTONS (Digital)            │
 │  ───────────────────                      ────────────────────         │
 │  X (Strafe): %+.4f                      %s       │
 │  Y (Forward):%+.4f                      %s       │
 │  Z (Turn):   %+.4f                      %s       │
 │  Slider:     %+.4f                                                   │
 │                                                                        │
 │  D-PAD:       %-14s                                │
 │                                                                        │
 │  D-PAD VISUAL:                                                         │
 │        %s                                                              │
 │      %s ○ %s                                                            │
 │        %s                                                              │
 │                                                                        │
 │     Buttons:  3:%s  4:%s  5:%s  6:%s                                   │
 └────────────────────────────────────────────────────────────────────────┘
`,
		m.x, btnRows[0],
		m.y, btnRows[1],
		m.z, btnRows[2],
		m.slider,
		povDisplay,
		getPOVSymbol(m.pov, "up", "▲"),
		getPOVSymbol(m.pov, "left", "◀"), getPOVSymbol(m.pov, "right", "▶"),
		getPOVSymbol(m.pov, "down", "▼"),
		getBtnSymbol(m.buttons[2]), getBtnSymbol(m.buttons[3]),
		getBtnSymbol(m.buttons[4]), getBtnSymbol(m.buttons[5]),
	)
}

func getPOVDisplaySimple(pov int) string {
	switch pov {
	case 0:
		return "○ CENTER"
	case 1:
		return "↑ UP"
	case 2:
		return "→ RIGHT"
	case 3:
		return "↗ UP-RIGHT"
	case 4:
		return "↓ DOWN"
	case 6:
		return "↘ DOWN-RIGHT"
	case 8:
		return "← LEFT"
	case 9:
		return "↖ UP-LEFT"
	case 12:
		return "↙ DOWN-LEFT"
	default:
		return fmt.Sprintf("? %d", pov)
	}
}

func getPOVSymbol(pov int, direction, symbol string) string {
	switch direction {
	case "up":
		if pov&1 != 0 {
			return symbol
		}
	case "right":
		if pov&2 != 0 {
			return symbol
		}
	case "down":
		if pov&4 != 0 {
			return symbol
		}
	case "left":
		if pov&8 != 0 {
			return symbol
		}
	}
	return " "
}

func getBtnSymbol(active bool) string {
	if active {
		return "●"
	}
	return "○"
}

// ============================================================================
// main
// ============================================================================

func main() {
	if err := sdl.Init(uint32(sdl.INIT_JOYSTICK)); err != nil {
		fmt.Println("SDL Başlatılamadı:", err)
		return
	}
	defer sdl.Quit()

	sdl.JoystickEventState(sdl.ENABLE)

	p := tea.NewProgram(initialModel())

	go func() {
		var joy *sdl.Joystick

		for {
			sdl.PumpEvents()

			numJoysticks := sdl.NumJoysticks()
			currentlyConnected := numJoysticks > 0

			if currentlyConnected && joy == nil {
				joy = sdl.JoystickOpen(0)
				if joy != nil {
					fmt.Println("DEBUG: Joystick bağlandı")
				}
			}

			if !currentlyConnected && joy != nil {
				joy.Close()
				joy = nil
				fmt.Println("DEBUG: Joystick çıkarıldı")
			}

			msg := JoystickMsg{
				Connected: currentlyConnected,
				X:         0,
				Y:         0,
				Z:         0,
				Slider:    0,
				POV:       0,
			}

			for i := 0; i < 12; i++ {
				msg.Buttons[i] = false
			}

			if currentlyConnected && joy != nil {
				msg.X = float64(joy.Axis(0)) / 32768.0
				msg.Y = -float64(joy.Axis(1)) / 32768.0
				msg.Z = float64(joy.Axis(2)) / 32768.0
				msg.Slider = float64(joy.Axis(3)) / 32768.0
				msg.POV = int(joy.Hat(0))

				for i := 0; i < 12; i++ {
					msg.Buttons[i] = (joy.Button(i) == 1)
				}
			}

			p.Send(msg)
			time.Sleep(50 * time.Millisecond)
		}
	}()

	if _, err := p.Run(); err != nil {
		fmt.Println("FATAL ERROR:", err)
	}
}
