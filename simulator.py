import constants as c
import pyrosim
import math


class SIMULATOR:
    def __init__(self, args, exploration_mode=False, wts=None, angles=None, pb=True, pp=False, capture=False):
        sim = pyrosim.Simulator(
            play_blind=pb,
            play_paused=pp,
            eval_time=c.sim_evalTime if exploration_mode is True else c.nn_evalTime,
            capture=capture)

        # args = list(18)
        #   args[0:6]: 'thigh_lens'
        #   args[6:12]: 'thigh_rads'
        #   args[12:18]: 'calf_lens'

        thigh_lens = args[0:6]
        thigh_rads = args[6:12]
        calf_lens = args[12:18]

        # /////////////////////////////////////////////////////////////////////
        # /////////////////////////////////////////////////////////////////////
        # body objects
        # /////////////////////////////////////////////////////////////////////
        # /////////////////////////////////////////////////////////////////////
        # ---------------------------------------------------------------------
        # main body
        body = sim.send_box(
            x=0,
            y=0,
            z=c.body_height / 2,
            length=c.body_length*1.2,
            width=c.body_width,
            height=c.body_height,
            mass=c.body_length * c.body_width * c.body_height * c.density,
            r=0.5,
            g=0.5,
            b=0.5)
        # ---------------------------------------------------------------------
        # ---------------------------------------------------------------------
        # joint linkers, workaround for 2 DOF joints
        # assigned counter-clockwise, from the right bottom leg
        linker_x = c.body_width / 2
        linker_z = list(map(lambda r: c.body_height/2 + r, thigh_rads))
        linkers = [
            sim.send_sphere(
                x=linker_x,
                y=c.hindleg_y,
                z=linker_z[0],
                radius=thigh_rads[0],
                mass=c.pseudo_mass,
                r=0.5,
                g=0.5,
                b=0.5),
            sim.send_sphere(
                x=linker_x,
                y=0,
                z=linker_z[1],
                radius=thigh_rads[1],
                mass=c.pseudo_mass,
                r=0.5,
                g=0.5,
                b=0.5),
            sim.send_sphere(
                x=linker_x,
                y=c.frontleg_y,
                z=linker_z[2],
                radius=thigh_rads[2],
                mass=c.pseudo_mass,
                r=0.5,
                g=0.5,
                b=0.5),
            sim.send_sphere(
                x=-linker_x,
                y=c.frontleg_y,
                z=linker_z[3],
                radius=thigh_rads[3],
                mass=c.pseudo_mass,
                r=0.5,
                g=0.5,
                b=0.5),
            sim.send_sphere(
                x=-linker_x,
                y=0,
                z=linker_z[4],
                radius=thigh_rads[4],
                mass=c.pseudo_mass,
                r=0.5,
                g=0.5,
                b=0.5),
            sim.send_sphere(
                x=-linker_x,
                y=c.hindleg_y,
                z=linker_z[5],
                radius=thigh_rads[5],
                mass=c.pseudo_mass,
                r=0.5,
                g=0.5,
                b=0.5)
        ]
        # ---------------------------------------------------------------------
        # ---------------------------------------------------------------------
        # lever_linkers
        # lever_linker_z = list(map(lambda r: c.body_height/2 - r, thigh_rads))
        # lever_linkers = [
        #     sim.send_sphere(
        #         x=linker_x,
        #         y=c.hindleg_y,
        #         z=lever_linker_z[0],
        #         radius=thigh_rads[0],
        #         mass=c.pseudo_mass,
        #         r=0.5,
        #         g=0.5,
        #         b=0.5),
        #     sim.send_sphere(
        #         x=linker_x,
        #         y=0,
        #         z=lever_linker_z[1],
        #         radius=thigh_rads[1],
        #         mass=c.pseudo_mass,
        #         r=0.5,
        #         g=0.5,
        #         b=0.5),
        #     sim.send_sphere(
        #         x=linker_x,
        #         y=c.frontleg_y,
        #         z=lever_linker_z[2],
        #         radius=thigh_rads[2],
        #         mass=c.pseudo_mass,
        #         r=0.5,
        #         g=0.5,
        #         b=0.5),
        #     sim.send_sphere(
        #         x=-linker_x,
        #         y=c.frontleg_y,
        #         z=lever_linker_z[3],
        #         radius=thigh_rads[3],
        #         mass=c.pseudo_mass,
        #         r=0.5,
        #         g=0.5,
        #         b=0.5),
        #     sim.send_sphere(
        #         x=-linker_x,
        #         y=0,
        #         z=lever_linker_z[4],
        #         radius=thigh_rads[4],
        #         mass=c.pseudo_mass,
        #         r=0.5,
        #         g=0.5,
        #         b=0.5),
        #     sim.send_sphere(
        #         x=-linker_x,
        #         y=c.hindleg_y,
        #         z=lever_linker_z[5],
        #         radius=thigh_rads[5],
        #         mass=c.pseudo_mass,
        #         r=0.5,
        #         g=0.5,
        #         b=0.5)
        # ]
        # ---------------------------------------------------------------------
        # ---------------------------------------------------------------------
        # thighs
        pythagoras_vl = list(
            map(lambda cl, r, lz: cl + r - lz, calf_lens, thigh_rads, linker_z))
        pythagoras_hl = list(map(lambda tl, vl: math.sqrt(
            math.pow(tl, 2) - math.pow(vl, 2)), thigh_lens, pythagoras_vl))
        thigh_x = list(map(lambda hl: hl / 2 + linker_x, pythagoras_hl))
        thigh_z = list(map(lambda vl, lz: vl / 2 + lz, pythagoras_vl, linker_z))
        thigh_angle = list(map(lambda tl, vl: math.asin(
            vl / tl), thigh_lens, pythagoras_vl))
        thigh_r3 = list(map(lambda a: math.tan(a), thigh_angle))
        thigh_mass = list(map(lambda tl, r: math.pi * math.pow(r, 2)
                              * tl * c.density, thigh_lens, thigh_rads))
        thighs = [
            sim.send_cylinder(
                x=thigh_x[0],
                y=c.hindleg_y,
                z=thigh_z[0],
                length=thigh_lens[0],
                radius=thigh_rads[0],
                r1=1,
                r2=0,
                r3=thigh_r3[0],
                mass=thigh_mass[0],
                r=0,
                g=0,
                b=1),
            sim.send_cylinder(
                x=thigh_x[1],
                y=0,
                z=thigh_z[1],
                length=thigh_lens[1],
                radius=thigh_rads[1],
                r1=1,
                r2=0,
                r3=thigh_r3[1],
                mass=thigh_mass[1],
                r=0,
                g=1,
                b=0),
            sim.send_cylinder(
                x=thigh_x[2],
                y=c.frontleg_y,
                z=thigh_z[2],
                length=thigh_lens[2],
                radius=thigh_rads[2],
                r1=1,
                r2=0,
                r3=thigh_r3[2],
                mass=thigh_mass[2],
                r=0,
                g=1,
                b=1),
            sim.send_cylinder(
                x=-thigh_x[3],
                y=c.frontleg_y,
                z=thigh_z[3],
                length=thigh_lens[3],
                radius=thigh_rads[3],
                r1=1,
                r2=0,
                r3=-thigh_r3[3],
                mass=thigh_mass[3],
                r=1,
                g=0,
                b=0),
            sim.send_cylinder(
                x=-thigh_x[4],
                y=0,
                z=thigh_z[4],
                length=thigh_lens[4],
                radius=thigh_rads[4],
                r1=1,
                r2=0,
                r3=-thigh_r3[4],
                mass=thigh_mass[4],
                r=1,
                g=0,
                b=1),
            sim.send_cylinder(
                x=-thigh_x[5],
                y=c.hindleg_y,
                z=thigh_z[5],
                length=thigh_lens[5],
                radius=thigh_rads[5],
                r1=1,
                r2=0,
                r3=-thigh_r3[5],
                mass=thigh_mass[5],
                r=1,
                g=1,
                b=0)
        ]
        # ---------------------------------------------------------------------
        # ---------------------------------------------------------------------
        # calves
        calf_x = list(map(lambda hl: hl + linker_x, pythagoras_hl))
        calf_z = list(map(lambda cl, r: cl / 2 + r,
                          calf_lens, thigh_rads))
        calf_mass = list(map(lambda cl, r: math.pi * math.pow(r, 2)
                             * cl * c.density, calf_lens, thigh_rads))
        calves = [
            sim.send_cylinder(
                x=calf_x[0],
                y=c.hindleg_y,
                z=calf_z[0],
                length=calf_lens[0],
                radius=thigh_rads[0],
                r1=0,
                r2=0,
                r3=1,
                mass=calf_mass[0],
                r=0,
                g=0,
                b=1),
            sim.send_cylinder(
                x=calf_x[1],
                y=0,
                z=calf_z[1],
                length=calf_lens[1],
                radius=thigh_rads[1],
                r1=0,
                r2=0,
                r3=1,
                mass=calf_mass[1],
                r=0,
                g=1,
                b=0),
            sim.send_cylinder(
                x=calf_x[2],
                y=c.frontleg_y,
                z=calf_z[2],
                length=calf_lens[2],
                radius=thigh_rads[2],
                r1=0,
                r2=0,
                r3=1,
                mass=calf_mass[2],
                r=0,
                g=1,
                b=1),
            sim.send_cylinder(
                x=-calf_x[3],
                y=c.frontleg_y,
                z=calf_z[3],
                length=calf_lens[3],
                radius=thigh_rads[3],
                r1=0,
                r2=0,
                r3=1,
                mass=calf_mass[3],
                r=1,
                g=0,
                b=0),
            sim.send_cylinder(
                x=-calf_x[4],
                y=0,
                z=calf_z[4],
                length=calf_lens[4],
                radius=thigh_rads[4],
                r1=0,
                r2=0,
                r3=1,
                mass=calf_mass[4],
                r=1,
                g=0,
                b=1),
            sim.send_cylinder(
                x=-calf_x[5],
                y=c.hindleg_y,
                z=calf_z[5],
                length=calf_lens[5],
                radius=thigh_rads[5],
                r1=0,
                r2=0,
                r3=1,
                mass=calf_mass[5],
                r=1,
                g=1,
                b=0)
        ]
        # ---------------------------------------------------------------------
        # ---------------------------------------------------------------------
        # levers
        # lever_z = list(map(lambda tz, r: tz - 2*r, thigh_z, thigh_rads))
        # levers = [
        #     sim.send_cylinder(
        #         x=thigh_x[0],
        #         y=c.hindleg_y,
        #         z=lever_z[0],
        #         mass=thigh_mass[0],
        #         length=thigh_lens[0],
        #         radius=thigh_rads[0],
        #         r1=1,
        #         r2=0,
        #         r3=thigh_r3[0],
        #         r=0,
        #         g=0,
        #         b=1),
        #     sim.send_cylinder(
        #         x=thigh_x[1],
        #         y=0,
        #         z=lever_z[1],
        #         mass=thigh_mass[1],
        #         length=thigh_lens[1],
        #         radius=thigh_rads[1],
        #         r1=1,
        #         r2=0,
        #         r3=thigh_r3[1],
        #         r=0,
        #         g=1,
        #         b=0),
        #     sim.send_cylinder(
        #         x=thigh_x[2],
        #         y=c.frontleg_y,
        #         z=lever_z[2],
        #         mass=thigh_mass[2],
        #         length=thigh_lens[2],
        #         radius=thigh_rads[2],
        #         r1=1,
        #         r2=0,
        #         r3=thigh_r3[2],
        #         r=0,
        #         g=1,
        #         b=1),
        #     sim.send_cylinder(
        #         x=-thigh_x[3],
        #         y=c.frontleg_y,
        #         z=lever_z[3],
        #         mass=thigh_mass[3],
        #         length=thigh_lens[3],
        #         radius=thigh_rads[3],
        #         r1=1,
        #         r2=0,
        #         r3=-thigh_r3[3],
        #         r=1,
        #         g=0,
        #         b=0),
        #     sim.send_cylinder(
        #         x=-thigh_x[4],
        #         y=0,
        #         z=lever_z[4],
        #         mass=thigh_mass[4],
        #         length=thigh_lens[4],
        #         radius=thigh_rads[4],
        #         r1=1,
        #         r2=0,
        #         r3=-thigh_r3[4],
        #         r=1,
        #         g=0,
        #         b=1),
        #     sim.send_cylinder(
        #         x=-thigh_x[5],
        #         y=c.hindleg_y,
        #         z=lever_z[5],
        #         mass=thigh_mass[5],
        #         length=thigh_lens[5],
        #         radius=thigh_rads[5],
        #         r1=1,
        #         r2=0,
        #         r3=-thigh_r3[5],
        #         r=1,
        #         g=1,
        #         b=0)
        # ]
        # ---------------------------------------------------------------------
        # /////////////////////////////////////////////////////////////////////
        # /////////////////////////////////////////////////////////////////////

        # /////////////////////////////////////////////////////////////////////
        # /////////////////////////////////////////////////////////////////////
        # joints
        # /////////////////////////////////////////////////////////////////////
        # /////////////////////////////////////////////////////////////////////
        # ---------------------------------------------------------------------
        # body-thigh joints, horizontal direction
        body_thigh_joints_h = [
            sim.send_hinge_joint(
                first_body_id=body,
                second_body_id=linkers[0],
                x=linker_x,
                y=c.hindleg_y,
                z=linker_z[0],
                lo=c.lo_h,
                hi=c.hi_h,
                n1=0,
                n2=1,
                n3=0),
            sim.send_hinge_joint(
                first_body_id=body,
                second_body_id=linkers[1],
                x=linker_x,
                y=0,
                z=linker_z[1],
                lo=c.lo_h,
                hi=c.hi_h,
                n1=0,
                n2=1,
                n3=0),
            sim.send_hinge_joint(
                first_body_id=body,
                second_body_id=linkers[2],
                x=linker_x,
                y=c.frontleg_y,
                z=linker_z[2],
                lo=c.lo_h,
                hi=c.hi_h,
                n1=0,
                n2=1,
                n3=0),
            sim.send_hinge_joint(
                first_body_id=body,
                second_body_id=linkers[3],
                x=-linker_x,
                y=c.frontleg_y,
                z=linker_z[3],
                lo=c.lo_h,
                hi=c.hi_h,
                n1=0,
                n2=1,
                n3=0),
            sim.send_hinge_joint(
                first_body_id=body,
                second_body_id=linkers[4],
                x=-linker_x,
                y=0,
                z=linker_z[4],
                lo=c.lo_h,
                hi=c.hi_h,
                n1=0,
                n2=1,
                n3=0),
            sim.send_hinge_joint(
                first_body_id=body,
                second_body_id=linkers[5],
                x=-linker_x,
                y=c.hindleg_y,
                z=linker_z[5],
                lo=c.lo_h,
                hi=c.hi_h,
                n1=0,
                n2=1,
                n3=0)
        ]
        # ---------------------------------------------------------------------
        # ---------------------------------------------------------------------
        # body-thigh joints, vertical direction
        body_thigh_joints_v = [
            sim.send_hinge_joint(
                first_body_id=linkers[0],
                second_body_id=thighs[0],
                x=linker_x,
                y=c.hindleg_y,
                z=linker_z[0],
                lo=c.lo_v,
                hi=c.hi_v,
                n1=0,
                n2=0,
                n3=1),
            sim.send_hinge_joint(
                first_body_id=linkers[1],
                second_body_id=thighs[1],
                x=linker_x,
                y=0,
                z=linker_z[1],
                lo=c.lo_v,
                hi=c.hi_v,
                n1=0,
                n2=0,
                n3=1),
            sim.send_hinge_joint(
                first_body_id=linkers[2],
                second_body_id=thighs[2],
                x=linker_x,
                y=c.frontleg_y,
                z=linker_z[2],
                lo=c.lo_v,
                hi=c.hi_v,
                n1=0,
                n2=0,
                n3=1),
            sim.send_hinge_joint(
                first_body_id=linkers[3],
                second_body_id=thighs[3],
                x=-linker_x,
                y=c.frontleg_y,
                z=linker_z[3],
                lo=c.lo_v,
                hi=c.hi_v,
                n1=0,
                n2=0,
                n3=1),
            sim.send_hinge_joint(
                first_body_id=linkers[4],
                second_body_id=thighs[4],
                x=-linker_x,
                y=0,
                z=linker_z[4],
                lo=c.lo_v,
                hi=c.hi_v,
                n1=0,
                n2=0,
                n3=1),
            sim.send_hinge_joint(
                first_body_id=linkers[5],
                second_body_id=thighs[5],
                x=-linker_x,
                y=c.hindleg_y,
                z=linker_z[5],
                lo=c.lo_v,
                hi=c.hi_v,
                n1=0,
                n2=0,
                n3=1)
        ]
        # ---------------------------------------------------------------------
        # ---------------------------------------------------------------------
        # body-lever joints, horizontal direction
        # sim.send_hinge_joint(
        #     first_body_id=body,
        #     second_body_id=lever_linkers[0],
        #     x=linker_x,
        #     y=c.hindleg_y,
        #     z=lever_linker_z[0],
        #     lo=c.lo_h,
        #     hi=c.hi_h,
        #     n1=0,
        #     n2=1,
        #     n3=0),
        # sim.send_hinge_joint(
        #     first_body_id=body,
        #     second_body_id=lever_linkers[1],
        #     x=linker_x,
        #     y=0,
        #     z=lever_linker_z[1],
        #     lo=c.lo_h,
        #     hi=c.hi_h,
        #     n1=0,
        #     n2=1,
        #     n3=0),
        # sim.send_hinge_joint(
        #     first_body_id=body,
        #     second_body_id=lever_linkers[2],
        #     x=linker_x,
        #     y=c.frontleg_y,
        #     z=lever_linker_z[2],
        #     lo=c.lo_h,
        #     hi=c.hi_h,
        #     n1=0,
        #     n2=1,
        #     n3=0),
        # sim.send_hinge_joint(
        #     first_body_id=body,
        #     second_body_id=lever_linkers[3],
        #     x=-linker_x,
        #     y=c.frontleg_y,
        #     z=lever_linker_z[3],
        #     lo=c.lo_h,
        #     hi=c.hi_h,
        #     n1=0,
        #     n2=1,
        #     n3=0),
        # sim.send_hinge_joint(
        #     first_body_id=body,
        #     second_body_id=lever_linkers[4],
        #     x=-linker_x,
        #     y=0,
        #     z=lever_linker_z[4],
        #     lo=c.lo_h,
        #     hi=c.hi_h,
        #     n1=0,
        #     n2=1,
        #     n3=0),
        # sim.send_hinge_joint(
        #     first_body_id=body,
        #     second_body_id=lever_linkers[5],
        #     x=-linker_x,
        #     y=c.hindleg_y,
        #     z=lever_linker_z[5],
        #     lo=c.lo_h,
        #     hi=c.hi_h,
        #     n1=0,
        #     n2=1,
        #     n3=0)
        # ---------------------------------------------------------------------
        # ---------------------------------------------------------------------
        # body-lever joints, vertical direction
        # sim.send_hinge_joint(
        #     first_body_id=lever_linkers[0],
        #     second_body_id=levers[0],
        #     x=linker_x,
        #     y=c.hindleg_y,
        #     z=lever_linker_z[0],
        #     lo=c.lo_v,
        #     hi=c.hi_v,
        #     n1=0,
        #     n2=0,
        #     n3=1),
        # sim.send_hinge_joint(
        #     first_body_id=lever_linkers[1],
        #     second_body_id=levers[1],
        #     x=linker_x,
        #     y=0,
        #     z=lever_linker_z[1],
        #     lo=c.lo_v,
        #     hi=c.hi_v,
        #     n1=0,
        #     n2=0,
        #     n3=1),
        # sim.send_hinge_joint(
        #     first_body_id=lever_linkers[2],
        #     second_body_id=levers[2],
        #     x=linker_x,
        #     y=c.frontleg_y,
        #     z=lever_linker_z[2],
        #     lo=c.lo_v,
        #     hi=c.hi_v,
        #     n1=0,
        #     n2=0,
        #     n3=1),
        # sim.send_hinge_joint(
        #     first_body_id=lever_linkers[3],
        #     second_body_id=levers[3],
        #     x=-linker_x,
        #     y=c.frontleg_y,
        #     z=lever_linker_z[3],
        #     lo=c.lo_v,
        #     hi=c.hi_v,
        #     n1=0,
        #     n2=0,
        #     n3=1),
        # sim.send_hinge_joint(
        #     first_body_id=lever_linkers[4],
        #     second_body_id=levers[4],
        #     x=-linker_x,
        #     y=0,
        #     z=lever_linker_z[4],
        #     lo=c.lo_v,
        #     hi=c.hi_v,
        #     n1=0,
        #     n2=0,
        #     n3=1),
        # sim.send_hinge_joint(
        #     first_body_id=lever_linkers[5],
        #     second_body_id=levers[5],
        #     x=-linker_x,
        #     y=c.hindleg_y,
        #     z=lever_linker_z[5],
        #     lo=c.lo_v,
        #     hi=c.hi_v,
        #     n1=0,
        #     n2=0,
        #     n3=1)
        # ---------------------------------------------------------------------
        # ---------------------------------------------------------------------
        # thigh-calf joints
        tc_joint_z = list(
            map(lambda cl, r: cl + r, calf_lens, thigh_rads))
        tc_joint_lo = -math.pi
        tc_joint_hi = math.pi
        sim.send_hinge_joint(
            first_body_id=thighs[0],
            second_body_id=calves[0],
            x=calf_x[0],
            y=c.hindleg_y,
            z=tc_joint_z[0],
            lo=tc_joint_lo,
            hi=tc_joint_hi,
            n1=0,
            n2=0,
            n3=1),
        sim.send_hinge_joint(
            first_body_id=thighs[1],
            second_body_id=calves[1],
            x=calf_x[1],
            y=0,
            z=tc_joint_z[1],
            lo=tc_joint_lo,
            hi=tc_joint_hi,
            n1=0,
            n2=0,
            n3=1),
        sim.send_hinge_joint(
            first_body_id=thighs[2],
            second_body_id=calves[2],
            x=calf_x[2],
            y=c.frontleg_y,
            z=tc_joint_z[2],
            lo=tc_joint_lo,
            hi=tc_joint_hi,
            n1=0,
            n2=0,
            n3=1),
        sim.send_hinge_joint(
            first_body_id=thighs[3],
            second_body_id=calves[3],
            x=-calf_x[3],
            y=c.frontleg_y,
            z=tc_joint_z[3],
            lo=tc_joint_lo,
            hi=tc_joint_hi,
            n1=0,
            n2=0,
            n3=1),
        sim.send_hinge_joint(
            first_body_id=thighs[4],
            second_body_id=calves[4],
            x=-calf_x[4],
            y=0,
            z=tc_joint_z[4],
            lo=tc_joint_lo,
            hi=tc_joint_hi,
            n1=0,
            n2=0,
            n3=1),
        sim.send_hinge_joint(
            first_body_id=thighs[5],
            second_body_id=calves[5],
            x=-calf_x[5],
            y=c.hindleg_y,
            z=tc_joint_z[5],
            lo=tc_joint_lo,
            hi=tc_joint_hi,
            n1=0,
            n2=0,
            n3=1)
        # ---------------------------------------------------------------------
        # ---------------------------------------------------------------------
        # lever-calf joints
        # lc_joint_z = list(map(lambda tcz, r: tcz - 2*r, tc_joint_z, thigh_rads))
        # sim.send_hinge_joint(
        #     first_body_id=levers[0],
        #     second_body_id=calves[0],
        #     x=calf_x[0],
        #     y=c.hindleg_y,
        #     z=lc_joint_z[0],
        #     lo=tc_joint_lo,
        #     hi=tc_joint_hi,
        #     n1=0,
        #     n2=0,
        #     n3=1),
        # sim.send_hinge_joint(
        #     first_body_id=levers[1],
        #     second_body_id=calves[1],
        #     x=calf_x[1],
        #     y=0,
        #     z=lc_joint_z[1],
        #     lo=tc_joint_lo,
        #     hi=tc_joint_hi,
        #     n1=0,
        #     n2=0,
        #     n3=1),
        # sim.send_hinge_joint(
        #     first_body_id=levers[2],
        #     second_body_id=calves[2],
        #     x=calf_x[2],
        #     y=c.frontleg_y,
        #     z=lc_joint_z[2],
        #     lo=tc_joint_lo,
        #     hi=tc_joint_hi,
        #     n1=0,
        #     n2=0,
        #     n3=1),
        # sim.send_hinge_joint(
        #     first_body_id=levers[3],
        #     second_body_id=calves[3],
        #     x=-calf_x[3],
        #     y=c.frontleg_y,
        #     z=lc_joint_z[3],
        #     lo=tc_joint_lo,
        #     hi=tc_joint_hi,
        #     n1=0,
        #     n2=0,
        #     n3=1),
        # sim.send_hinge_joint(
        #     first_body_id=levers[4],
        #     second_body_id=calves[4],
        #     x=-calf_x[4],
        #     y=0,
        #     z=lc_joint_z[4],
        #     lo=tc_joint_lo,
        #     hi=tc_joint_hi,
        #     n1=0,
        #     n2=0,
        #     n3=1),
        # sim.send_hinge_joint(
        #     first_body_id=levers[5],
        #     second_body_id=calves[5],
        #     x=-calf_x[5],
        #     y=c.hindleg_y,
        #     z=lc_joint_z[5],
        #     lo=tc_joint_lo,
        #     hi=tc_joint_hi,
        #     n1=0,
        #     n2=0,
        #     n3=1)
        # ---------------------------------------------------------------------
        # /////////////////////////////////////////////////////////////////////
        # /////////////////////////////////////////////////////////////////////
        # move joints according to angles
        if exploration_mode is True:
            assert len(angles) == 12

            sensors = [None] * 12
            # 6 horizontal joint angle sensors
            for i in range(6):
                sensors[i] = sim.send_proprioceptive_sensor(joint_id=body_thigh_joints_h[i])
            # 6 vertical joint angle sensors
            for i in range(6):
                sensors[6+i] = sim.send_proprioceptive_sensor(joint_id=body_thigh_joints_v[i])

            sensor_neurons = [None] * 12
            for i in range(12):
                sensor_neurons[i] = sim.send_sensor_neuron(sensor_id=sensors[i])

            motor_neurons = [None] * 12
            # 6 horizontal joint motors
            for i in range(6):
                motor_neurons[i] = sim.send_motor_neuron(joint_id=body_thigh_joints_h[i])
            # 6 vertical joint motors
            for i in range(6):
                motor_neurons[6+i] = sim.send_motor_neuron(joint_id=body_thigh_joints_v[i])

            # FIXME: 1 bias neuron?
            bias_neurons = [None] * 12
            for i in range(12):
                bias_neurons[i] = sim.send_bias_neuron()

            for i in range(12):
                sim.send_synapse(source_neuron_id=sensor_neurons[i],
                                 target_neuron_id=motor_neurons[i],
                                 weight=-10)
                sim.send_synapse(source_neuron_id=bias_neurons[i],
                                 target_neuron_id=motor_neurons[i],
                                 weight=10*angles[i])
        # move joints by ctrnn
        else:
            sensors = [None] * 12
            # 6 ground touch sensors
            for i in range(6):
                sensors[i] = sim.send_touch_sensor(body_id=calves[i])
            # 6 locheck proprioceptive sensors
            for i in range(6):
                sensors[6+i] = sim.send_proprioceptive_sensor(
                                joint_id=body_thigh_joints_h[i],
                                locheck=True,
                                lo=c.lo_h)

            sensor_neurons = [None] * 12
            for i in range(12):
                sensor_neurons[i] = sim.send_sensor_neuron(sensor_id=sensors[i])

            motor_neurons = [None] * 12
            # 6 horizontal joint motors
            for i in range(6):
                motor_neurons[i] = sim.send_motor_neuron(joint_id=body_thigh_joints_h[i])
            # 6 vertical joint motors
            for i in range(6):
                motor_neurons[6+i] = sim.send_motor_neuron(joint_id=body_thigh_joints_v[i])

            sim.send_ctrnn(connections=c.ctrnn_connections,
                           wts=wts, taus=c.taus,
                           input_neuron_ids=sensor_neurons,
                           output_neuron_ids=motor_neurons)

        self.sim = sim
        self.position_sensor = sim.send_position_sensor(body_id=body)
