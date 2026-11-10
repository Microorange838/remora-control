"""Reduced-order closed-loop simulation: descent trajectory, hydrodynamic drag and a
Pymunk/pygame loop driven by the six-input controller.

Used during development to sanity-check the controller; it is not part of the deployed
system. Any entry point that produces a figure must import mlp26.config first.

The nominal descent is prescribed rather than simulated: the head height and its first
two derivatives are read from a cubic Bezier curve through four control points, so that
the controller sees a smooth HEIGHT / HEIGHT_VELOCITY / HEIGHT_ACCELERATION input. The
body itself is a two-link model (tail + head) hinged at the neck, with the neck driven by
a damped rotary spring whose rest angle is the controller output; hydrodynamic drag is
applied to both links.
"""

import matplotlib.pyplot as plt
import numpy as np
import pygame
import pymunk.pygame_util


def create_smooth_descent_from_steps(control_points, dt):
    """Build a smooth descent (height, velocity, acceleration) from four control points.

    Args:
        control_points: four (step, height) pairs, e.g.
            [(0, 7.5), (100, 7.0), (300, 1.5), (500, 0.0)]
        dt: time step (s)

    Returns:
        (h_func, v_func, a_func): height, velocity and acceleration of time
    """
    if len(control_points) != 4:
        raise ValueError("exactly four control points are required for the Bezier curve")

    P0, P1, P2, P3 = control_points
    t0, h0 = P0  # step, height
    t1, h1 = P1
    t2, h2 = P2
    t3, h3 = P3

    # steps -> seconds
    t_start = t0 * dt
    t_end = t3 * dt
    duration = t_end - t_start

    # Bezier: B(tau) = (1-tau)^3 P0 + 3(1-tau)^2 tau P1 + 3(1-tau) tau^2 P2 + tau^3 P3
    def bezier_curve(tau, h0, h1, h2, h3):
        """Cubic Bezier curve, tau in [0, 1]."""
        coef0 = (1 - tau)**3
        coef1 = 3 * (1 - tau)**2 * tau
        coef2 = 3 * (1 - tau) * tau**2
        coef3 = tau**3
        return coef0 * h0 + coef1 * h1 + coef2 * h2 + coef3 * h3

    def bezier_velocity(tau, h0, h1, h2, h3, duration):
        """First derivative of the Bezier curve."""
        # B'(tau) = 3(1-tau)^2 (P1-P0) + 6(1-tau) tau (P2-P1) + 3 tau^2 (P3-P2)
        dtau_dt = 1.0 / duration  # dtau/dt
        dh_dtau = (3 * (1 - tau)**2 * (h1 - h0) +
                   6 * (1 - tau) * tau * (h2 - h1) +
                   3 * tau**2 * (h3 - h2))
        return dh_dtau * dtau_dt

    def bezier_acceleration(tau, h0, h1, h2, h3, duration):
        """Second derivative of the Bezier curve."""
        # B''(tau) = 6(1-tau)(P2-2P1+P0) + 6 tau (P3-2P2+P1)
        dtau_dt = 1.0 / duration
        d2h_dtau2 = (6 * (1 - tau) * (h2 - 2*h1 + h0) +
                     6 * tau * (h3 - 2*h2 + h1))
        return d2h_dtau2 * dtau_dt**2

    def h_func(t):
        """Height at time t."""
        if t < t_start:
            return h0
        elif t <= t_end:
            tau = (t - t_start) / duration
            return float(bezier_curve(tau, h0, h1, h2, h3))
        else:
            return h3

    def v_func(t):
        """Vertical velocity at time t."""
        if t < t_start or t > t_end:
            return 0.0
        else:
            tau = (t - t_start) / duration
            return float(bezier_velocity(tau, h0, h1, h2, h3, duration))

    def a_func(t):
        """Vertical acceleration at time t."""
        if t < t_start or t > t_end:
            return 0.0
        else:
            tau = (t - t_start) / duration
            return float(bezier_acceleration(tau, h0, h1, h2, h3, duration))

    print("\nsmooth descent built (Bezier curve)")
    print("control points:")
    for i, (step, height) in enumerate(control_points):
        t_point = step * dt
        print(f"  P{i}: step={step:4d}, t={t_point:6.3f}s, h={height:5.2f}m")
    print(f"total time: {duration:.3f}s, total steps: {int(t3 - t0)}")
    print(f"initial height: {h_func(t_start):.3f} m")
    print(f"final height:   {h_func(t_end):.3f} m")
    print(f"initial velocity: {v_func(t_start + 0.001):.3f} m/s")
    print(f"final velocity:   {v_func(t_end - 0.001):.3f} m/s")
    print(f"mean velocity: {(h3 - h0)/duration:.3f} m/s")

    return h_func, v_func, a_func


def apply_hydrodynamic_drag(body, length, C_n=1.2, C_t=0.02, rho=1000, diameter=0.1):
    """Apply hydrodynamic drag to a link, split into normal and tangential components.

    Args:
        body: pymunk rigid body
        length: physical length of the link (m)
        C_n: normal drag coefficient (perpendicular to the link), typically 1.0-1.5
        C_t: tangential drag coefficient (along the link), typically 0.01-0.05
        rho: water density (kg/m^3)
        diameter: link diameter (m)
    """
    # link axes (unit vectors)
    angle = body.angle
    tangent = pymunk.Vec2d(np.cos(angle), np.sin(angle))
    normal = pymunk.Vec2d(-np.sin(angle), np.cos(angle))

    # split the velocity into normal and tangential components
    v = body.velocity
    v_n = v.dot(normal) * normal
    v_t = v.dot(tangent) * tangent

    # drag magnitude F = 0.5 * rho * C * A * v^2, projected areas: the normal one is
    # length * diameter, the tangential one is the circular cross-section pi*(d/2)^2
    A_n = length * diameter
    A_t = np.pi * (diameter/2)**2

    v_n_mag = v_n.length
    v_t_mag = v_t.length

    F_n = pymunk.Vec2d(0, 0)
    F_t = pymunk.Vec2d(0, 0)

    if v_n_mag > 1e-6:  # guard against division by zero
        F_n = -0.5 * rho * C_n * A_n * v_n_mag * v_n
    if v_t_mag > 1e-6:
        F_t = -0.5 * rho * C_t * A_t * v_t_mag * v_t

    # apply at the centre of mass
    total_force = F_n + F_t
    body.apply_force_at_world_point(total_force, body.position)

    # angular damping (reduced model): torque proportional to omega^2 * L^3
    angular_drag_coeff = 0.5 * rho * C_n * (length**3) * diameter / 12
    omega = body.angular_velocity
    torque = -angular_drag_coeff * omega * abs(omega)
    body.torque += torque


def apply_distributed_drag(body, length, n_segments=10, C_n=1.2, C_t=0.02, rho=1000, diameter=0.1):
    """Apply the hydrodynamic drag segment by segment along the link.

    Args:
        n_segments: number of segments the link is divided into
    """
    angle = body.angle
    tangent = pymunk.Vec2d(np.cos(angle), np.sin(angle))
    normal = pymunk.Vec2d(-np.sin(angle), np.cos(angle))

    segment_length = length / n_segments
    A_n_seg = segment_length * diameter
    A_t_seg = np.pi * (diameter/2)**2 * (segment_length / length)

    for i in range(n_segments):
        # local position of the segment (relative to the centre of mass, which sits at
        # length/2, so the link spans (-length/2, 0) to (length/2, 0))
        local_x = -length/2 + (i + 0.5) * segment_length
        local_pos = pymunk.Vec2d(local_x, 0)

        world_pos = body.local_to_world(local_pos)

        # velocity at the segment = centre-of-mass velocity + omega x r
        r = local_pos
        v_point = body.velocity + pymunk.Vec2d(-r.y, r.x) * body.angular_velocity

        v_n = v_point.dot(normal) * normal
        v_t = v_point.dot(tangent) * tangent

        v_n_mag = v_n.length
        v_t_mag = v_t.length

        F_n = pymunk.Vec2d(0, 0)
        F_t = pymunk.Vec2d(0, 0)

        if v_n_mag > 1e-6:
            F_n = -0.5 * rho * C_n * A_n_seg * v_n_mag * v_n
        if v_t_mag > 1e-6:
            F_t = -0.5 * rho * C_t * A_t_seg * v_t_mag * v_t

        segment_force = F_n + F_t

        # applying off-centre also produces the corresponding torque
        body.apply_force_at_world_point(segment_force, world_pos)


# model/scaler_x/scaler_y are passed in rather than read from module globals: the
# simulation drives the six-input controller.
# Note the x2 factor on the predicted neck angle and the upper-only clamp at 60 degrees:
# both are carried over from the original simulation and are not physically justified.
def plot_simu_bio_simulation_pymunk_v2(model, scaler_x, scaler_y, dt=0.004, use_drag=True, drag_mode='simple'):
    pygame.init()
    screen = pygame.display.set_mode((600, 600))
    draw_options = pymunk.pygame_util.DrawOptions(screen)  # pymunk's own renderer
    # --- 1. environment ---
    space = pymunk.Space()
    space.gravity = (0, 0)

    # --- 2. two-link body (head and tail share a hinge at the neck) ---
    m_h, m_t = 1.0, 2.5  # head and tail masses (kg)
    l_h, l_t = 2.0, 5.0  # physical link lengths (m)
    scale = 20  # display scale

    # hinge position in world coordinates
    joint_pos = pymunk.Vec2d(300, 300)

    # === TAIL ===
    # Moment of inertia from the parallel axis theorem, since the link rotates about its
    # end rather than its centre of mass: I = I_cm + m*d^2 with d = l_t/2.
    i_t_cm = pymunk.moment_for_box(m_t, (l_t, 0.5))  # about the centre of mass
    i_t = i_t_cm + m_t * (l_t/2)**2                  # shifted to the end

    tail = pymunk.Body(m_t, i_t)
    # the tail centre of mass is l_t/2 to the left of the hinge
    tail.angle = np.radians(-40)  # set the angle first
    # the position must account for the rotation so that the hinge lands on joint_pos
    tail_offset = pymunk.Vec2d(l_t/2, 0).rotated(tail.angle)
    tail.position = joint_pos - tail_offset

    # vertices are relative to the centre of mass and extend to the right, putting the
    # hinge at the right end of the link
    w, h = l_t*scale, 10
    tail_vertices = [
        (0, -h/2),      # bottom left (hinge)
        (w, -h/2),      # bottom right
        (w, h/2),       # top right
        (0, h/2)        # top left (hinge)
    ]
    tail_shape = pymunk.Poly(tail, tail_vertices)
    tail_shape.color = (0, 0, 255, 255)  # blue
    tail_shape.filter = pymunk.ShapeFilter(group=1)  # same group: no self-collision

    # === HEAD ===
    i_h_cm = pymunk.moment_for_box(m_h, (l_h, 0.5))
    i_h = i_h_cm + m_h * (l_h/2)**2

    head = pymunk.Body(m_h, i_h)
    # the head centre of mass is l_h/2 to the right of the hinge
    head.angle = np.radians(-40)  # initial dive angle of 40 degrees
    offset_vec = pymunk.Vec2d(l_h/2, 0).rotated(head.angle)
    head.position = joint_pos + offset_vec

    # vertices extend to the left, putting the hinge at the left end of the link
    w, h = l_h*scale, 10
    head_vertices = [
        (-w, -h/2),     # bottom left
        (0, -h/2),      # bottom right (hinge)
        (0, h/2),       # top right (hinge)
        (-w, h/2)       # top left
    ]
    head_shape = pymunk.Poly(head, head_vertices)
    head_shape.color = (255, 0, 0, 255)  # red
    head_shape.filter = pymunk.ShapeFilter(group=1)

    # === CONSTRAINTS ===
    # PivotJoint: the hinge in each body's local coordinates (l_t/2 for the tail,
    # -l_h/2 for the head)
    pivot = pymunk.PivotJoint(tail, head,
                              (l_t/2, 0),   # tail local coordinates
                              (-l_h/2, 0))  # head local coordinates

    # A damped rotary spring stands in for the neck muscle: the torque is
    # stiffness * angular deviation from rest_angle, with damping to reduce oscillation.
    neck_spring = pymunk.DampedRotarySpring(
        tail, head,
        rest_angle=np.radians(0),  # target NECK angle (rad): relative head-tail angle
        stiffness=10000,
        damping=800
    )

    space.add(tail, tail_shape, head, head_shape, pivot, neck_spring)

    # === hydrodynamic parameters ===
    water_rho = 1000  # water density, kg/m^3
    diameter = 0.1    # link diameter (m)
    C_n = 1.2         # normal drag coefficient
    C_t = 0.02        # tangential drag coefficient (much smaller than the normal one)

    print("\nhydrodynamic parameters")
    print(f"drag enabled: {use_drag}")
    print(f"mode: {drag_mode}")
    if use_drag:
        print(f"normal drag coefficient C_n: {C_n}")
        print(f"tangential drag coefficient C_t: {C_t}")
        print(f"link diameter: {diameter}m")
        print(f"water density: {water_rho} kg/m^3")

    # --- 3. simulation state ---
    # Prescribed descent through four (step, height) control points: P0 the start,
    # P1 the initial descent rate, P2 the approach to the surface, P3 the end.
    control_points = [
        (0, 7.5),      # start
        (50, 0),       # control point 1
        (200, 0.5),    # control point 2
        (500, 0.0)     # end
    ]
    h_func, v_func, a_func = create_smooth_descent_from_steps(control_points, dt)

    t = 0.0  # simulation time
    curr_h = h_func(t)
    v_h = v_func(t)
    a_h = a_func(t)

    last_v_d_deg = 0.0  # previous disc rate, for the angular acceleration
    history = {'h': [], 'disc': [], 'neck': [], 'v_d': [], 't': []}

    print("running the full-angle alignment simulation...")

    for i in range(500 + 100):  # total steps plus margin
        # --- a. read the pymunk state, in degrees ---
        curr_disc_deg = -np.degrees(head.angle)
        v_d_deg = -np.degrees(head.angular_velocity)  # deg/s
        a_d_deg = (v_d_deg - last_v_d_deg) / dt       # deg/s^2

        # --- b. assemble the six inputs, in the same units as the CSV ---
        # [DISC, HEIGHT, DISC_VEL, DISC_ACCEL, HEIGHT_VEL, HEIGHT_ACCEL]
        state = np.array([[
            curr_disc_deg, curr_h,
            v_d_deg, a_d_deg,
            v_h, a_h
        ]])

        # --- c. controller decision (degrees) ---
        state_scaled = scaler_x.transform(state)
        pred_scaled = model.predict(state_scaled, verbose=0)
        target_neck_deg = scaler_y.inverse_transform(pred_scaled)[0][0] * 2

        target_neck_deg = min(target_neck_deg, 60)

        # --- d. feed the decision back to pymunk, in radians ---
        neck_spring.rest_angle = np.radians(-target_neck_deg)

        # --- d2. apply the hydrodynamic drag ---
        if use_drag:
            if drag_mode == 'simple':
                # whole-link drag applied at the centre of mass
                apply_hydrodynamic_drag(tail, l_t, C_n, C_t, water_rho, diameter)
                apply_hydrodynamic_drag(head, l_h, C_n, C_t, water_rho, diameter)
            elif drag_mode == 'distributed':
                # drag integrated along the link
                apply_distributed_drag(tail, l_t, n_segments=8, C_n=C_n, C_t=C_t, rho=water_rho, diameter=diameter)
                apply_distributed_drag(head, l_h, n_segments=5, C_n=C_n, C_t=C_t, rho=water_rho, diameter=diameter)

        # --- e. step the physics and advance the prescribed height ---
        space.step(dt)
        t += dt
        curr_h = h_func(t)
        v_h = v_func(t)
        a_h = a_func(t)

        last_v_d_deg = v_d_deg
        history['h'].append(curr_h)
        history['disc'].append(curr_disc_deg)
        history['neck'].append(target_neck_deg)
        history['v_d'].append(v_d_deg)
        history['t'].append(t)

        # pygame event handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return

        # render
        screen.fill((255, 255, 255))
        space.debug_draw(draw_options)
        pygame.display.flip()

        if curr_h <= 0:
            break

    pygame.quit()

    # --- 4. figures ---
    fig, axes = plt.subplots(2, 3, figsize=(18, 10), layout="compressed")

    axes[0, 0].plot(history['t'], history['disc'], 'b-o', markersize=2)
    axes[0, 0].set_title('Disc angle vs time')
    axes[0, 0].set_xlabel('time t (s)')
    axes[0, 0].set_ylabel('disc (deg)')
    axes[0, 0].grid(True, alpha=0.3)

    axes[0, 1].plot(history['neck'], label='neck angle command', color='orange')
    axes[0, 1].set_title('Controller output')
    axes[0, 1].legend()

    axes[0, 2].plot(history['v_d'], label='angular velocity (deg/s)')
    axes[0, 2].axhline(y=0, color='black', linestyle='--')
    axes[0, 2].set_title('Angular velocity')
    axes[0, 2].legend()

    axes[1, 0].plot(history['t'], history['h'], 'b-', linewidth=2)
    axes[1, 0].set_title('Height vs time')
    axes[1, 0].set_xlabel('time t (s)')
    axes[1, 0].set_ylabel('height (m)')
    axes[1, 0].grid(True, alpha=0.3)

    height_velocity = [v_func(t) for t in history['t']]
    axes[1, 1].plot(history['t'], height_velocity, 'g-', linewidth=2)
    axes[1, 1].set_title('Vertical velocity vs time')
    axes[1, 1].set_xlabel('time t (s)')
    axes[1, 1].set_ylabel('vertical velocity (m/s)')
    axes[1, 1].axhline(y=0, color='black', linestyle='--')
    axes[1, 1].grid(True, alpha=0.3)

    axes[1, 2].axis('off')

    plt.show()
