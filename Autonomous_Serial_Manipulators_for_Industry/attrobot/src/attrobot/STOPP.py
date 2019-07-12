#! /usr/bin/env python

from math import sin, cos, tan, asin, pi, ceil
import numpy as np
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
class STOPP:

    __d_min = 0
    __d_min_cruise = 0
    __d_max_cruise = 0
    __dv_min_cruise = 0
    __d_min_to_vmax = 0
    __d_min_for_amax_pulse = 0
    __d_min_to_vf = 0
    mode = ""
    __dv_min = 0
    acc_tol = 0.0000001
    jerk_tol = 0.000001
    d_tol = 0.0000001
    t_tol = 0.0000001
    vel_tol = 0.001

    def __init__(self,p1,p2,j_max,a_max,v_max,time_step, v1, a1,pf, interpolation=False, point_by_point=True):

        self.interpolation = interpolation
        self.point_by_point = point_by_point
        self.pf = pf
        self.time_step = time_step
        self.j_max = j_max
        self.j_max_final = -1*j_max
        self.__j_max = j_max
        self.a_max = a_max
        self.__a_final = 0
        self.__a_max = a_max
        self.v_max = v_max
        self.__v_max = v_max
        self.__v_1 = v1
        self.__a_1 = a1
        if p1 > p2:
            self.p1 = p2
            self.p2 = p1
            self.flip_data = True
        else:
            self.p1 = p1
            self.p2 = p2
            self.flip_data = False
        self.distance = abs(self.p2-self.p1)

        self.t = 0
        #self.n = asin(0.0505)
        self.n = 0
        self.J = self.j_max * (1+np.cos(self.n))/pi
        self.J_final = self.j_max_final*(1+np.cos(self.n))/pi
        self.__t_a = max((self.a_max - self.__a_1) / self.J, 0.0)
        self.__t_d = max((self.__a_final - self.a_max) / self.J_final, 0.0)
        self.__v_a = 0.5*self.__t_a*(self.a_max + self.__a_1) + self.__v_1
        self.__v_d = 0.5*self.__t_d*(self.a_max + self.__a_final)
        self.__t_b = max((self.v_max - self.__v_a - self.__v_d) / self.a_max, 0.0)
        self.__v_b = self.__t_b * self.a_max
        self.__v_2 = self.__v_a + self.__v_b + self.__v_d
        self.__theta_a = (np.power(self.__t_a,3)*self.J/6) + 0.5*self.__a_1*np.power(self.__t_a,2) +\
                         self.__v_1*self.__t_a + self.p1
        self.__theta_b = np.power(self.__t_b,2)*self.a_max/2 + self.__v_a*self.__t_b
        self.__theta_d = (np.power(self.__t_d,2)*self.a_max/2) + np.power(self.__t_d,3)*self.J_final/6 \
                         + (self.__v_b + self.__v_a)*self.__t_d
        self.__t_f = 0
        self.__theta_f = 0
        self.total_time = self.__t_a + self.__t_b + self.__t_d + self.__t_f
        self.__Acc_Flag = True

    def Get_time_to_start_acc_ramp_down(self):
        return self.__t_a+self.__t_b

    def Calculate_theta_d(self, v1, am, j, vf=0, af=0, using_af=True, choose_min=True):

        if using_af:
            td = (af - am) / j
            vd = 0.5 * td * (am + af)
        else:
            vd = vf-v1
            a = 0.5*j
            b = am
            c = -vd
            coeff = [a, b, c]
            ##print("coeff = ", coeff)
            td_roots = np.roots(coeff)
            td_roots = td_roots[np.
            isclose(td_roots.imag, 0.0)]
            td_roots = td_roots.real
            ##print(td_roots.real)
            index = np.array(np.where(td_roots > 0))
            if index.size == 0:
                return 0,0,0,0
            if choose_min:
                td = np.min(td_roots[index])
            else:
                td = np.max(td_roots[index])
            af = j * td + am
        theta_d = (np.power(td, 2) * am / 2) + np.power(td, 3) * j / 6 + v1 * td
        return td,af,vd,theta_d

    def Calculate_theta_b(self, vb, am, v1):

        tb = max(vb / am, 0.0)
        theta_b = np.power(tb, 2) * am / 2 + v1 * tb
        return tb,theta_b

    def Calculate_theta_a(self,am,a1,J,v1,p1):

        ta = (am-a1)/J
        va = 0.5*(am+a1)*ta + v1
        theta_a = ((ta**2) * a1 / 2) + (ta**3) * J / 6 + v1 * ta + p1
        return ta,theta_a,va

    def Get_total_time(self):

        v_pulse = self.__v_a + self.__v_d + self.vel_tol
        if (v_pulse > self.v_max) and (self.__a_1>0):
            td = (self.__a_1/self.J)
            self.total_time = td + self.distance/self.v_max
        elif (v_pulse <= self.v_max) and (self.__a_1>0):
            self.total_time = self.total_time + \
                              self.distance/self.v_max
        elif self.__v_1+self.vel_tol >= self.v_max:
            self.total_time = self.distance/self.v_max
        elif self.__v_1 - self.vel_tol <=0:
            self.total_time = self.__t_a + self.__t_b + self.__t_d + (self.distance/self.v_max)
        else:
            self.total_time = 0


    def Change_ta(self, ta):

        self.__t_a = ta
        self.a_max = self.J * self.__t_a + self.__a_1
        self.__v_a = 0.5 * self.__t_a * (self.a_max + self.__a_1) + self.__v_1
        self.__theta_a = (np.power(self.__t_a, 3) * self.J / 6) + self.__v_1 * self.__t_a \
                         + ( np.power(self.__t_a, 2) * self.__a_1 / 2) + self.p1

    def Change_tb(self, tb):

        self.__t_b = tb
        self.__v_b = self.__t_b * self.a_max
        self.__theta_b = np.power(self.__t_b, 2) * self.a_max / 2 + self.__v_a*self.__t_b

    def Calculate_tb(self, d, v1, a1):

        a = 0.5 * a1
        b = v1
        c = -d
        coeff = [a, b, c]
        # #print("coeff = ", coeff)
        tb_roots = np.roots(coeff)
        tb_roots = tb_roots[np.isclose(tb_roots.imag, 0.0)]
        tb_roots = tb_roots.real
        # #print(tb_roots.real)
        index = np.where(tb_roots > 0)
        tb = np.min(tb_roots[index])
        v_cruise = tb * a1
        theta_b = 0.5 * a1 * (tb ** 2) + v1 * tb

        return tb, theta_b, v_cruise

    def Calculate_cruising_distance(self, v_f):
        # deceleration to a certain velocity mode requires it for decision making
        tb = (v_f - self.__v_a) / self.a_max
        cruising_distance = np.power(tb, 2) * self.a_max / 2 + self.__v_a*tb
        return cruising_distance

    def Change_vd(self, v_d):
        # deceleration to a certain velocity mode requires it for decision making
        self.__v_d = v_d
        under_root = 2*self.__v_d*self.J_final + np.power(self.a_max,2)
        if under_root < 0:
            ##print("ERROR:: NO SOLUTION FOR A_FINAL")
            self.mode = "None"
            return False
        self.__a_final = np.sqrt(under_root)
        if self.__a_final > self.a_max and self.__a_final <= self.__a_max :
            self.__a_final = -1 * self.__a_final
        elif self.__a_final > self.__a_max:
            ##print("ERROR:: A_FINAL EXCEEDED ACC LIMIT")
            self.mode = "None"
            return False
        self.__t_d = (self.__a_final - self.a_max)/self.J_final
        self.__theta_d = (np.power(self.__t_d,2)*self.a_max/2) + np.power(self.__t_d,3)*self.J_final/6 \
                         + (self.__v_b + self.__v_a)*self.__t_d

    def Change_td(self, td):
        self.__t_d = td
        self.__a_final = self.__t_d * self.J_final + self.a_max
        self.__v_d = 0.5 * self.__t_d * (self.a_max + self.__a_final)
        self.__theta_d = (np.power(self.__t_d,2)*self.a_max/2) + np.power(self.__t_d,3)*self.J_final/6 \
                         + (self.__v_b + self.__v_a)*self.__t_d

    def Change_J_ta(self, J, ta): # deceleration ramp up mode
        self.J = J
        self.__t_a = ta
        self.j_max = self.J*pi/(1+np.cos(self.n))
        self.a_max = self.J * self.__t_a + self.__a_1
        self.__v_a = 0.5 * self.__t_a * (self.a_max + self.__a_1) + self.__v_1
        self.__theta_a = (np.power(self.__t_a, 3) * self.J / 6) + self.__v_1 * self.__t_a \
                         + (np.power(self.__t_a, 2) * self.__a_1 / 2) + self.p1

    def Change_J_final_td(self, J_final, td):
        self.J_final = J_final
        self.__t_d = td
        self.j_max_final = self.J_final * pi / (1 + np.cos(self.n))
        self.__a_final = self.__t_d * self.J_final + self.a_max
        self.__v_d = 0.5 * self.__t_d * (self.a_max + self.__a_final)
        self.__theta_d = (np.power(self.__t_d,2)*self.a_max/2) + np.power(self.__t_d,3)*self.J_final/6 \
                         + (self.__v_b + self.__v_a)*self.__t_d

    def Change_a_final_td(self, a_final, td): # dec-ramp-up-cruise-ramp-down
        self.__a_final = a_final
        self.__t_d = td
        self.J_final = (self.__a_final - self.a_max)/self.__t_d
        self.j_max_final = self.J_final * pi / (1 + np.cos(self.n))
        self.__v_d = 0.5 * self.__t_d * (self.a_max + self.__a_final)
        self.__theta_d = (np.power(self.__t_d,2)*self.a_max/2) + np.power(self.__t_d,3)*self.J_final/6 \
                         + (self.__v_b + self.__v_a)*self.__t_d

    def Change_max_joint_jerk(self,j_max_new, ramp_down=False):

        if ramp_down:
            self.j_max_final = j_max_new
            self.J_final = self.j_max_final * (1 + np.cos(self.n)) / pi
            td = (self.__a_final - self.a_max) / self.J_final
            self.Change_td(td)

        else:
            self.j_max = j_max_new
            self.J = self.j_max * (1 + np.cos(self.n)) / pi
            ta = (self.a_max-self.__a_1) / self.J
            self.Change_ta(ta)

    def Change_max_joint_acceleration(self,a_max_new,ramp_down=False):

        #self.a_max[self.max_index] = a_max_new
        if ramp_down:
            self.__a_final = a_max_new
            td = (self.__a_final - self.a_max) / self.J_final
            self.Change_td(td)
        else:
            self.a_max = a_max_new
            ta = (self.a_max - self.__a_1) / self.J
            self.Change_ta(ta)

    def Pseudo_ramp_up_to_v(self, a1, v1, vf):

        a = self.J / 2
        b = a1
        c = v1-vf
        coeff = [a, b, c]
        ##print("coeff = ", coeff)
        ta_roots = np.roots(coeff)
        ta_roots = ta_roots[np.isclose(ta_roots.imag, 0.0)]
        ta_roots = ta_roots.real
        ##print(ta_roots.real)
        index = np.where(ta_roots > 0)
        ta = np.min(ta_roots[index])
        am = self.J*ta + a1
        theta_a = (np.power(ta, 3) * self.J / 6) + v1 * ta + (np.power(ta, 2) * a1 / 2)
        return ta,theta_a,am

    def Ramp_up_to_d(self):

        a = self.J / 6
        b = self.__a_1 / 2
        c = self.__v_1
        l = - self.distance
        coeff = [a, b, c, l]
        ##print("coeff = ", coeff)
        ta_roots = np.roots(coeff)
        ta_roots = ta_roots[np.isclose(ta_roots.imag, 0.0, 0.001, 0.0001)]
        ta_roots = ta_roots.real
        ##print(ta_roots.real)
        index = np.array(np.where(ta_roots >= 0))
        ta = 0
        if index.size > 0:
            ta = np.min(ta_roots[index])
        self.Change_ta(ta)
        if (ta-0.0000001 < 0)or(index.size<0):

            tb,theta_b,v_cruise = self.Calculate_tb(self.distance,self.__v_1,self.__a_1)
            self.__t_b = tb
            self.__v_b = v_cruise
            self.__theta_b = theta_b
        else:
            self.__t_b = 0

        self.__t_d = 0
        self.__t_f = 0

    def Pulse_to_d(self,d,v1,a1,J):

        coeff = [1 / (J ** 2), 0, (- (a1 ** 2) + 2 * J * v1) / (J ** 2),
                  -(3 * d * (J ** 2) + 3 * v1 * J * a1 - (a1 ** 3)) / (3 * (J ** 2))]
        # #print("coeff = ", coeff)
        am_roots = np.roots(coeff)
        am_roots = am_roots[np.isclose(am_roots.imag, 0.0, 0.001, 0.0001)]
        am_roots = am_roots.real
        ##print(ta_roots.real)
        index = np.array(np.where(am_roots >= 0.0))
        if index.size > 0:
            am = np.min(am_roots[index])
            ta = (am-a1)/J
            td = am/J
        else:
            ta = 0
            td = 0

        return ta, td

    def Cruise_ramp_down_to_d(self, d, v1, a1, J):

        td = a1/J

        a = 0.5 * a1
        b = v1 + a1*td
        c = -d + v1*td - J*(td**3)/6.0 + a1*(td**2)/2.0
        coeff = [a, b, c]
        # #print("coeff = ", coeff)
        tb_roots = np.roots(coeff)
        tb_roots = tb_roots[np.isclose(tb_roots.imag, 0.0)]
        tb_roots = tb_roots.real
        # #print(tb_roots.real)
        index = np.where(tb_roots > 0)
        tb = np.min(tb_roots[index])

        return tb,td

    def Ramp_down_dwell_to_d(self,Jf,am,v1,d=-100,using_vf = False):

        ta = 0
        va = 0
        theta_a = 0
        a_max = 0
        a = Jf / 6
        b = am / 2
        c = v1
        if d > -100:
            l = -1 * d
        else:
            d = self.distance
        l = -1 * d
        coeff = [a, b, c, l]
        ##print("coeff = ", coeff)
        td_roots = np.roots(coeff)
        td_roots = td_roots[np.isclose(td_roots.imag, 0.0,0.001,0.001)]
        td_roots = td_roots.real
        ##print(td_roots.real)
        index = np.array(np.where(td_roots > 0))
        if index.size == 0:
            ##print("ERROR:NO VALID SOLUTION FOR td")
            self.mode = 'None'
            return 0,0,0,0,0,0,0,0,0,0,0
        td = np.min(td_roots[index])
        af = Jf*td+am
        if using_vf:
            if af <= -self.__a_max:
                af = -self.__a_max
                if (am-self.acc_tol)<=-self.__a_max:
                    td = 0
                    vd = 0
                    theta_d = 0
                else:
                    td, _, vd, theta_d = self.Calculate_theta_d(v1, am, Jf, af=af)

                tf,theta_f,v_cruise = self.Calculate_tb(d-theta_d, v1+vd, af)

                va = (-(af)**2)/(2*(-Jf))
                if (v_cruise + vd + v1 + va) < 0:
                    v_cruise = v1+vd+va
                    tf = v_cruise / af
                    theta_f = 0.5*af*(tf**2) + (v1+vd)*tf
                    theta_a = d - theta_d - theta_f
                    a = -Jf / 6
                    b = am / 2
                    c = v1
                    l = -theta_a
                    coeff = [a, b, c, l]
                    ##print("coeff = ", coeff)
                    ta_roots = np.roots(coeff)
                    ta_roots = ta_roots[np.isclose(ta_roots.imag, 0.0)]
                    ta_roots = ta_roots.real
                    ##print(ta_roots.real)
                    index = np.array(np.where(ta_roots >= 0))
                    ta = np.min(ta_roots[index])
                    a_max = (-Jf)*ta + am
            else:
                tf=0
                theta_f = 0
                v_cruise = 0
                td, _, vd, theta_d = self.Calculate_theta_d(v1, am, Jf, af=af)
        else:
            if af < 0:
                af = 0
                td, _, vd, theta_d = self.Calculate_theta_d(v1, am, Jf, af=af)
                v_cruise = v1+vd
                tf = (d-theta_d)/(v_cruise)
                theta_f = tf*(v_cruise)
            else:
                tf=0
                theta_f = 0
                v_cruise = 0
                td, _, vd, theta_d = self.Calculate_theta_d(v1, am, Jf, af=af)

        return td,theta_d,vd,af,tf,theta_f,v_cruise,ta,theta_a,va,a_max

    def Ramp_down_to_v(self, v1, vf, a1, Jf, af=0, using_af=False, choose_min=True):

        if (vf<= self.vel_tol):
            a_peak = -1*self.__a_max
            v_pulse = ((a1**2)-2*(a_peak**2)+2*(-Jf)*v1)/(2*(-Jf))
            if (vf + v_pulse) <= 0 and ((a1+self.acc_tol)>=-self.__a_max):
                a_peak = -np.sqrt(0.5*((a1**2)+2*(-Jf)*v1-2*(-Jf)*vf))
                v_pulse = ((a1**2)-2*(a_peak**2)+2*(-Jf)*v1)/(2*(-Jf))
                tb = 0
                vb = 0
                theta_b = 0
                af = a_peak
                td = max((a_peak - a1) / Jf,0.0)
                vd = 0.5 * (a_peak + a1) * td
            else:
                af = a_peak
                td = max((a_peak - a1) / Jf,0.0)
                vd = 0.5 * (a_peak + a1) * td
                vb = vf - v_pulse
                tb, theta_b = self.Calculate_theta_b(vb, -self.__a_max, v1 + vd)

            theta_d = Jf*(td**3)/6 + a1*(td**2)/2 + v1*td
            ta = (-a_peak) / (-Jf)
            va = 0.5*(a_peak)*ta
            theta_a = ((ta ** 3) * -Jf / 6) + (v1 + vd + vb) * ta + ((ta ** 2) * a_peak / 2)
            min_distance = theta_a+theta_b+theta_d

        else:
            if using_af:
                td, af, vd, theta_d = self.Calculate_theta_d(v1, a1, Jf, af=af)
                if td == 0:
                    return 0,0,0,0,0,0,0,0,0,0,0
                tb = 0
                theta_b = 0
                vb = 0
                min_distance = theta_d
            else:
                td, af, vd, theta_d = self.Calculate_theta_d(v1, a1, Jf, af=-self.__a_max)
                if td == 0:
                    return 0,0,0,0,0,0,0,0,0,0,0
                if vf >= (v1 + vd):
                    td, af, vd, theta_d = self.Calculate_theta_d(v1, a1, Jf, vf=vf, using_af=False, choose_min=False)
                    if td == 0:
                        return 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                    min_distance = theta_d
                    tb = 0
                    theta_b = 0
                    vb = 0
                else:
                    vb = vf - (v1 + vd)
                    tb, theta_b = self.Calculate_theta_b(vb, -self.__a_max, v1 + vd)
                    min_distance = theta_d + theta_b
            ta = 0
            theta_a = 0
            va = 0
        return min_distance,td,theta_d,vd,af,tb,theta_b,vb,ta,theta_a,va

    def Ramp_up_to_d_and_v(self, a1, v1, v_f, d):

        a = a1/6
        b = (v_f+(2*v1/3))
        c = -d

        coeff = [a, b, c]
        ##print("coeff = ", coeff)
        ta_roots = np.roots(coeff)
        ta_roots = ta_roots[np.isclose(ta_roots.imag, 0.0)]
        ta_roots = ta_roots.real
        ##print(ta_roots.real)
        index = np.where(ta_roots > 0)
        ta = np.min(ta_roots[index])
        J = 2*(v_f-a1*ta-v1)/(ta**2)
        j = J * pi / (1 + np.cos(self.n))


        self.J = J
        self.__t_a = ta
        self.j_max = self.J*pi/(1+np.cos(self.n))
        self.a_max = self.J * self.__t_a + a1
        self.__v_a = v_f - v1
        self.__theta_a = d
        self.__t_b = 0
        self.__t_d = 0
        self.__t_f = 0


        return j,ta

    def Calculate_d_min_to_vpeak_then_to_vf(self, vf, v_max=False):

        if v_max:
            v_peak = self.__v_max
            d_to_v_peak = self.__d_max_cruise
        else:
            self.t_d_max, _, self.v_d_max, self.theta_d_max = self.Calculate_theta_d(self.v_a_max, self.__a_max, self.J_final, af=0)
            v_peak = self.v_a_max + self.v_d_max
            self.v_2_max = v_peak
            self.t_b_max = 0
            self.v_b_max = 0
            self.theta_b_max = 0
            d_to_v_peak = self.theta_a_max + self.theta_d_max
            self.a_max_max = self.__a_max
            self.a_final_max = 0

        distance_to_vf, td, theta_d, vd, af, tb, theta_b, vb, ta, theta_a, va = \
            self.Ramp_down_to_v(v_peak,vf,0,self.J_final)
        min_distance = d_to_v_peak + distance_to_vf

        if self.distance >= (min_distance-self.d_tol):
            self.__a_final = float(af)
            self.a_max = 0.0
            self.__t_d = td
            self.__v_d = vd
            self.__theta_d = theta_d
            self.__v_2 = vf
            self.__theta_a = theta_a
            self.__t_a = ta
            self.__v_a = va
            self.__theta_b = theta_b
            self.__t_b = tb
            self.__v_b = vb
            self.__theta_f = 0
            self.__t_f = 0
            if v_max:
                self.__d_min_to_vmax = min_distance
            else:
                self.__d_min_for_amax_pulse = min_distance
            return True

        else:
            return False

    def Pulse_to_vmax(self,v1,a1,J):

        am = np.sqrt(0.5 * (2 * J * (self.v_max - v1) + (a1 ** 2)))
        ta = (am - a1)/J
        td = am/J

        return ta, td

    def Pulse_to_v(self,J,v1,vf,a1,af,p1): # ramp_up_then_down, theta_a and va are absolute

        if (vf<= self.vel_tol):
            under_root = 0.5 * (2 * J * (vf - v1) + (a1 ** 2) + 2*(af ** 2))
        else:
            under_root = 0.5*(2*J*(vf-v1) + (a1**2) + (af**2))

        if under_root < 0:
            self.mode = 'None'
            if (vf<= self.vel_tol):
                return 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
            return 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
        else:
            am = np.array([np.sqrt(under_root), -np.sqrt(under_root)])

            if (am[0]>=a1):
                ##print("am[0] = ", am[0])
                ta,theta_a,va = self.Calculate_theta_a(am,a1,J,v1,p1)
                if (vf<= self.vel_tol):
                    td1, _, vd1, theta_d1 = self.Calculate_theta_d(va, am, -J, af=0)
                    v_peak = va + vd1
                    if am[0] > self.__a_max:
                        am[0] = self.__a_max
                        ta, theta_a, va = self.Calculate_theta_a(am, a1, J, v1, p1)
                        tb1 = (v_peak-va-0.5*(-(am**2)/-J))/am
                        vb1 = tb1*am
                        theta_b1 = vb1*tb1 + va*tb1
                        td1, _, vd1, theta_d1 = self.Calculate_theta_d(va+vb1, am, -J, af=0)
                    else :
                        tb1 = np.array([0,0])
                        vb1 = np.array([0,0])
                        theta_b1 = np.array([0,0])
                    min_distance, td2, theta_d2, vd2, af, \
                    tb, theta_b, vb, ta2, theta_a2, va2 = \
                        self.Ramp_down_to_v(v_peak[0], vf, 0, -J, choose_min=False)
                    d_covered = theta_a + theta_b1 + theta_d1 + theta_d2 + theta_b + theta_a2 - self.p1
                else:
                    if am[0] > self.__a_max:
                        td1, _, vd1, theta_d1 = self.Calculate_theta_d(va, am, -J, af=0)
                        v_peak = va+vd1
                        am[0] = self.__a_max
                        ta, theta_a, va = self.Calculate_theta_a(am, a1, J, v1, p1)
                        tb1 = (v_peak-va-0.5*(-(am**2)/-J))/am
                        vb1 = tb1*am
                        theta_b1 = vb1*tb1 + va*tb1
                    else :
                        tb1 = np.array([0,0])
                        vb1 = np.array([0,0])
                        theta_b1 = np.array([0,0])
                    td, _, vd, theta_d = self.Calculate_theta_d(va+vb1, am, -J, af=af)
                    d_covered = theta_a + theta_b1 + theta_d - self.p1
                if ((d_covered[0]-self.d_tol) <= self.distance)and((d_covered[0]+self.d_tol)>=0):
                    if not((vf<= self.vel_tol)):
                        td1, _, vd1, theta_d1 = self.Calculate_theta_d(va+vb1, am, -J, af=0)
                        td2 = td-td1
                        theta_d2 = theta_d-theta_d1
                        vd2 = vd-vd1
                    v_cruise=va+vb1+vd1
                    tf = (self.distance-d_covered[0])/v_cruise
                    theta_f = v_cruise*tf

                    if (vf<= self.vel_tol):

                        return ta[0], theta_a[0], va[0], tb1[0], vb1[0], theta_b1[0], td1[0], theta_d1[0], vd1[0],\
                               tf[0], theta_f[0], v_cruise[
                            0], td2, theta_d2, vd2, ta2, theta_a2, va2

                    return ta[0],theta_a[0],va[0], tb1[0], vb1[0], theta_b1[0], td1[0],theta_d1[0],vd1[0],\
                           tf[0],theta_f[0],v_cruise[0],td2[0],theta_d2[0],vd2[0]

                elif (am[1]>=a1) and (am[1]<=self.__a_max) and (d_covered[1]>self.distance):
                    self.mode = 'up-down-dwell-solver'
                    if (vf<= self.vel_tol):
                        return 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                    return 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
                else: # means d_covered[0] > distance and sol2 is eihter not applicable or d_covered[1] < distance
                    self.mode = 'up-down-dwell-solver'
                    if (vf<= self.vel_tol):
                        return 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                    return 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
            else:
                self.mode = 'None'
                if (vf<= self.vel_tol):
                    return 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                return 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

    def Go_to_v_max_using_lower_a_peak(self, a1, v1, J, p1):

        a_peak = np.sqrt(J * (self.v_max - v1) + 0.5 * (a1 ** 2))
        ta = max((a_peak - a1) / J, 0.0)
        va = 0.5 * ta * (a_peak + a1) + v1
        theta_a = (np.power(ta, 3) * J / 6) + v1 * ta + (np.power(ta, 2) * a1 / 2) + p1

        td,af,vd,theta_d = self.Calculate_theta_d(va, a_peak, -J, af=0)

        return ta,theta_a,va,a_peak,td,theta_d,vd,af

    def Calculate_d_min(self):

        self.__d_min = self.__theta_a - self.p1
        self.__dv_min = self.__v_a + self.__v_d - self.__v_1
        self.__dv_min_cruise = self.__v_a + self.__v_b - self.__v_1
        self.__d_min_cruise = self.__theta_a + self.__theta_b - self.p1
        self.__d_max_cruise = self.__theta_a + self.__theta_b + self.__theta_d - self.p1


    def Max_joint_whole_profile(self,v_f=0, velocity_mode=False):

        if not velocity_mode:

            State = True
            self.Calculate_d_min()
            td, af, vd, theta_d = self.Calculate_theta_d(self.__v_a, self.a_max, self.J_final)
            d_min_pulse = self.__d_min + theta_d
            if (self.__v_1+self.vel_tol) >= self.v_max:
                self.__v_2 = self.v_max
                self.mode = "Linear"
                return self.mode

            if (self.__dv_min+self.__v_1-self.vel_tol) >= self.v_max:
                #print("WARNING:: The min dv to reach a_max is grater than v_max")
                #print("WARNING:: Switching to Reduced acceleration profile")
                State = False


            self.mode = 'None'
            if (self.distance >= self.__d_max_cruise) and State :

                self.mode = 'General Case'
                self.__t_f = ((self.distance-self.__d_max_cruise)/self.__v_2)
                self.__theta_f = self.__t_f*self.__v_2

            elif (self.distance >= d_min_pulse) and (self.distance < self.__d_max_cruise) and State :

                self.mode = 'No Acceleration Dwell and Reduced Cruise Velocity'
                tb, td = self.Cruise_ramp_down_to_d(self.distance-self.__d_min, self.__v_a, self.a_max, self.J)
                self.Change_tb(tb)
                self.Change_td(td)
                self.__t_f = 0
                self.__v_2 = self.__v_a + self.__v_b + self.__v_d

            elif ((self.distance < d_min_pulse) or (not State)):


                self.mode = 'Acceleration Pulse with Reduced Max Acceleration'
                ta, td = self.Pulse_to_d(self.distance,self.__v_1,self.__a_1,self.J)
                self.Change_ta(ta)
                self.Change_tb(0.0)
                self.Change_td(td)
                self.__t_f = 0
                self.__v_2 = self.__v_a + self.__v_d

                if (self.__v_2 > self.v_max) and self.__Acc_Flag:

                    self.__v_2 = self.v_max
                    ta, td = self.Pulse_to_vmax(self.__v_1, self.__a_1, self.J)
                    self.Change_ta(ta)
                    self.Change_tb(0.0)
                    self.Change_td(td)
                    self.__t_f = ((self.distance-self.__theta_a-self.__theta_d+self.p1)/self.__v_2)
                    self.__theta_f = self.__t_f * self.__v_2
                    self.mode = "Reduced Acceleration with dwell on vmax"

            self.v_max = self.__v_2
            return self.mode


    def Max_joint_decision_making(self, v_f=0, velocity_mode=False):

        if not velocity_mode:

            State = True

            if self.__v_1 >= (self.v_max - self.vel_tol):
                self.mode = "Linear"
                self.__t_a = 0
                self.__t_b = 0
                self.__t_d = 0
                return self.mode

            self.Calculate_d_min()

            if (self.__dv_min + self.__v_1) >= self.v_max:
                ##print("Ramp Down, To Avoid Exceeding V_max")
                State = False
            self.mode = 'None'

            if (self.distance > self.__d_max_cruise) and State :

                self.mode = 'ramp-up-cruise-ramp-down-dwell'
                self.__v_2 = self.v_max
                self.__t_b = ((self.__v_2 - self.__v_a - self.__v_d)/self.a_max)
                self.__t_f = ((self.distance-self.__d_max_cruise)/self.__v_2)
                self.__theta_f = self.__v_2*self.__t_f

            elif (self.distance > self.__d_min_cruise) and (self.distance <= self.__d_max_cruise) and State:

                self.mode = 'ramp-up-cruise-ramp-down'

                a = self.J_final/6
                b = self.a_max/2
                c = self.__v_b + self.__v_a
                l = -(self.distance - self.__d_min_cruise)
                coeff = [a, b, c, l]
                ##print("coeff = ", coeff)
                td_roots = np.roots(coeff)
                td_roots = td_roots.real
                ##print(td_roots.real)
                index = np.where(td_roots > 0)
                td = np.min(td_roots[index])
                self.Change_td(td)
                self.__t_f = 0
                self.__v_2 = self.__v_a + self.__v_b + self.__v_d

            elif (self.distance > self.__d_min) and (self.distance <= self.__d_min_cruise) and State :

                self.mode = 'ramp-up-cruise'
                ##print("dmin = ", self.__d_min)
                a = 0.5*self.a_max
                b = self.__v_a
                c = self.__d_min-self.distance
                coeff = [a,b,c]
                tb_roots = np.roots(coeff)
                ##print(tb_roots)
                tb_roots = tb_roots[np.isclose(tb_roots.imag,0.0)]
                tb_roots = tb_roots.real
                ##print("tb_roots = ", tb_roots)
                index = np.array(np.where(tb_roots >= 0.0))
                if index.size == 0 :
                    ##print("ERROR:: NO SOLUTION FOR tb")
                    self.mode = 'None'
                    return self.mode
                self.Change_tb(tb_roots[index[0]][0])
                self.__v_2 = self.__v_a + self.__v_b
                self.__t_d = 0
                self.__t_f = 0

            elif ((self.distance <= self.__d_min) or (not State)) and (self.distance > 0) :


                if State:
                    self.mode = 'acc-ramp-up'
                    self.Ramp_up_to_d()
                else :

                    a_peak = np.sqrt(self.J*(self.v_max-self.__v_1) + 0.5*(self.__a_1**2))
                    if a_peak > self.__a_1:

                        self.Change_max_joint_acceleration(a_peak)
                        self.Change_td(self.a_max/self.J)

                        if self.distance <= (self.__theta_a-self.p1):
                            self.Ramp_up_to_d()
                            self.mode = 'ramp-up'
                            return self.mode
                        elif (self.distance <= (self.__theta_a+self.__theta_d-self.p1)):
                                self.__t_d,self.__theta_d,self.__v_d,self.__a_final,\
                                self.__t_f,self.__theta_f,self.__v_2,_,_,_,_\
                                    = self.Ramp_down_dwell_to_d(self.J_final,self.a_max,self.__v_a,
                                                                d = (self.distance - (self.__theta_a-self.p1)))
                                if ((self.__theta_a-self.p1) - self.d_tol) < 0:
                                    self.__t_a = 0
                                self.__t_b = 0
                                self.mode = 'ramp-up-down'
                                return self.mode
                        else:
                            self.__v_2 = self.v_max
                            self.__t_f = (self.distance - (self.__theta_a+self.__theta_d-self.p1))/self.__v_2
                            self.__theta_f = self.__v_2*self.__t_f
                            self.__t_b = 0
                            if ((self.__theta_a - self.p1) - self.d_tol) < 0:
                                self.__t_a = 0
                            self.mode = 'ramp-up-down-dwell'
                            return self.mode
                    self.mode = 'ramp-down-to-d'
                    self.Change_max_joint_acceleration(self.__a_1)
                    self.__t_d,self.__theta_d,self.__v_d,self.__a_final,self.__t_f,self.__theta_f,self.__v_2,_,_,_,_\
                        = self.Ramp_down_dwell_to_d(self.J_final,self.a_max,self.__v_a)
                    self.__t_b = 0
                    self.__t_a = 0

            return self.mode

    def Up_down_solver(self):

        x = fsolve(self.equations, np.array([0, 0]))
        am = x[0] + self.__a_1
        af_max = x[-1] + am
        ##print("am = ", am)
        ##print("af = ", af_max)
        self.a_max_max = am
        self.t_a_max = (am - self.__a_1) / self.J
        self.v_a_max = 0.5 * (am + self.__a_1) * self.t_a_max + self.__v_1
        self.theta_a_max = (np.power(self.t_a_max, 3) * self.J / 6) + self.__v_1 * self.t_a_max \
                           + (np.power(self.t_a_max, 2) * self.__a_1 / 2) + self.p1

        if (af_max < -self.__a_max)or(self.v_final<self.vel_tol):
            self.t_d_max, _, self.v_d_max, self.theta_d_max = \
                self.Calculate_theta_d(self.v_a_max, self.a_max_max, self.J_final, af=0)
            self.__d_min_to_vf, self.__t_d, self.__theta_d, self.__v_d, self.__a_final,\
            self.__t_b, self.__theta_b, self.__v_b, self.__t_a, self.__theta_a, self.__v_a =\
                self.Ramp_down_to_v(self.v_a_max+self.v_d_max, self.v_final, 0, self.J_final)
            self.__v_2 = self.v_final
            self.a_max = 0
            self.a_final_max = 0
            self.v_2_max = self.v_a_max + self.v_d_max
            self.t_f_max = (self.distance-self.__d_min_to_vf-self.theta_a_max-self.theta_d_max+float(self.p1))/\
                           (self.v_2_max)
            if self.t_f_max > 0 or (self.v_final <= self.vel_tol):
                self.theta_f_max = self.v_2_max*self.t_f_max
            else:
                self.t_d_max = 0
                self.t_f_max = 0
                self.__t_a = 0
                self.__t_d, self.__theta_d, self.__v_d, self.__a_final, \
                self.__t_b, self.__theta_b, self.__v_b = \
                    self.Ramp_down_dwell_to_d(self.J_final,self.a_max_max,self.v_a_max,
                                              d=self.distance-self.theta_a_max+self.p1,using_vf=True)
            self.__t_f = 0
            self.t_b_max = 0
            self.mode = 'dec-ramp-up-ramp-down-cruise'
            return self.mode
            #return self.up_down_dwell_solver(self.v_final)

        else:
            self.a_final_max = af_max
            self.__d_min_to_vf, self.t_d_max, self.theta_d_max, self.v_d_max, self.a_final_max, \
            self.__t_b, self.__theta_b, self.__v_b, self.__t_a, self.__theta_a, self.__v_a = \
                self.Ramp_down_to_v(self.v_a_max, self.v_final, self.a_max_max, self.J_final,af=af_max, using_af=True)
            if (self.distance != (self.theta_a_max+self.__d_min_to_vf-self.p1-self.d_tol)) and (self.v_final>self.vel_tol):
                self.__t_d = 0
                self.t_f_max = 0
                self.__t_a = 0
                self.t_d_max, self.theta_d_max, self.v_d_max, self.a_final_max, \
                self.__t_b, self.__theta_b, self.__v_b = \
                    self.Ramp_down_dwell_to_d(self.J_final,self.a_max_max,self.v_a_max,
                                              d=self.distance-self.theta_a_max+self.p1,using_vf=True)
            if self.t_d_max == 0:

                self.mode = 'None'
                return self.mode

            self.v_2_max = self.v_a_max + self.v_d_max
            self.__t_d = 0
            self.__t_f = 0
            self.t_b_max = 0
            self.t_f_max = 0
            self.mode = 'dec-ramp-up-ramp-down'
            return  self.mode

    def up_down_dwell_solver(self, v_f):
        am, tb = self.up_down_dwell(self.__v_1, v_f, self.__a_1, -self.__a_max, self.J, self.p1)
        if am==0 and tb==0:
            return 'yalla b2a'
        ##print("am = ", am)
        ##print("tb = ", tb)
        self.a_max_max = am
        self.a_final_max = -self.__a_max
        self.t_a_max, self.theta_a_max, self.v_a_max = \
            self.Calculate_theta_a(am, self.__a_1, self.J, self.__v_1, self.p1)

        if (tb < 0) and (am >= 0):

            self.t_d_max, _, self.v_d_max, self.theta_d_max = \
                self.Calculate_theta_d(self.v_a_max, self.a_max_max, self.J_final, af=0)

            self.__d_min_to_vf, self.__t_d, self.__theta_d, self.__v_d, self.__a_final, \
            self.__t_b, self.__theta_b, self.__v_b, self.__t_a, self.__theta_a, self.__v_a = \
                self.Ramp_down_to_v(self.v_a_max + self.v_d_max, v_f, 0, self.J_final, choose_min=False)

            self.a_final_max = 0
            self.v_2_max = self.v_a_max + self.v_d_max
            self.t_f_max = (self.distance - self.theta_a_max - self.theta_d_max - self.__d_min_to_vf + self.p1) / (
                self.v_2_max)
            self.theta_f_max = self.v_2_max * self.t_f_max

        else:
            self.t_d_max, _, self.v_d_max, self.theta_d_max = \
                self.Calculate_theta_d(self.v_a_max, self.a_max_max, self.J_final, af=self.a_final_max)
            self.__t_b = tb
            self.__v_b = tb * self.a_final_max
            self.__theta_b = 0.5 * self.__v_b * tb + (self.v_a_max + self.v_d_max) * tb
            v_at_af = self.v_a_max + self.v_d_max + self.__v_b
            if (v_f<= self.vel_tol):
                self.__t_a, self.__theta_a, self.__v_a = \
                    self.Calculate_theta_a(0, self.a_final_max, self.J, v_at_af, 0)
                self.__v_a = self.__v_a - v_at_af
            else:
                self.__t_a = 0

        self.__v_2 = v_f
        self.a_max = 0
        self.t_b_max = 0
        self.__t_f = 0
        self.mode = 'up-down-dwell-solver'
        return self.mode

    def equations(self,p):
        a, b = p
        k = a-b
        f1 = k**2 - 2*(b**2) - 2*b*self.__a_1 - 2*self.J*(self.v_final - self.__v_1)
        f2 = k**3 + 3*self.__a_1*(k**2) + 6*self.J*self.__v_1*k + 2*(b**3) - 6*(self.J**2)*self.distance
        return (f1, f2)

    def up_down_dwell(self,v1,vf,a1,af,J,p1):

        if (vf<= self.vel_tol):
            coeff = [ -1/(2*(J**2)*af), 1/(J**2), -(- (a1**2) + (af**2) + 2*J*v1)/(2*(J**2)*af),
                      (- (a1**2) + 2*J*v1)/(J**2),
                      -(3*(a1**4) - 8*(a1**3)*af + 12*(J**2)*(v1**2) - 12*(J**2)*(vf**2) - 6*(a1**2)*(af**2) +
                        24*(J**2)*af*self.distance - 24*(J**2)*af*p1 - 12*J*(a1**2)*v1 + 12*J*(af**2)*v1 +
                        12*J*(af**2)*vf + 24*J*a1*af*v1)/(24*(J**2)*af)]
        else:
            coeff = [ -1/(2*(J**2)*af), 1/(J**2), -(- (a1**2) + (af**2) + 2*J*v1)/(2*(J**2)*af),
                  (- (a1**2) + 2*J*v1)/(J**2),
                  (8*(a1**3)*af - 3*(a1**4) + (af**4) - 12*(J**2)*(v1**2) +
                   12*(J**2)*(vf**2) + 6*(a1**2)*(af**2) - 24*(J**2)*af*self.distance
                   + 24*(J**2)*af*p1 + 12*J*(a1**2)*v1 - 12*J*(af**2)*v1 - 24*J*a1*af*v1)/(24*(J**2)*af)]

        am_roots = np.roots(coeff)
        ##print(am_roots)
        am_roots = am_roots[np.isclose(am_roots.imag, 0.0)]
        am_roots = am_roots.real
        ##print("am_roots = ", am_roots)
        index = np.array(np.where(np.logical_and(am_roots >= a1, am_roots <= self.__a_max)))
        if index.size == 0:
            ##print("ERROR:: NO SOLUTION FOR am")
            ##print("a1 = ",a1)
            ##print("v1 = ", v1)
            ##print("vf = ", vf)
            ##print("d = ", self.distance)
            if (abs(a1)-self.acc_tol) <= 0 :
                self.__d_min_to_vf, self.__t_d, self.__theta_d, self.__v_d, self.__a_final, \
                self.__t_b, self.__theta_b, self.__v_b, self.__t_a, self.__theta_a, self.__v_a = \
                    self.Ramp_down_to_v(v1, vf, a1, self.J_final)
                self.t_a_max = 0
                self.t_b_max = 0
                self.a_max = 0
                self.t_d_max = 0
                self.a_final_max = a1
                self.t_f_max = (self.distance-self.__d_min_to_vf)/v1
                self.theta_f_max = v1*self.t_f_max
                self.v_2_max = v1
                self.__t_f = 0
            else:
                self.Up_down_solver()

            return 0,0

            # self.mode = 'None'
            # return self.mode

        am = np.max(am_roots[index])
        if (vf<= self.vel_tol):
            tb = (2 * J * (vf - v1) + (a1 ** 2) + 2*(af ** 2) - 2 * (am ** 2)) / (2 * J * af)
        else:
            tb = (2 * J * (vf - v1) + (a1**2) + (af**2) - 2 * (am**2)) / (2 * J * af)
        return am,tb

    def Interpolate(self,t0,tf,x0,xf,v0,vf,a0,af):

        size = round(((tf-t0)/self.time_step)) + 1
        time_array = np.linspace(t0, tf, size)
        if vf == v0:
            thetas_row = np.linspace(x0, xf, size)
            dthetas_row = np.ones(thetas_row.shape)*v0
            ddthetas_row = np.zeros(thetas_row.shape)
            data = np.array([thetas_row,dthetas_row,ddthetas_row])
            return data,time_array

        i00 = (t0**5)
        i01 = (t0**4)
        i02 = (t0**3)
        i03 = (t0**2)
        i04 = t0
        i05 = 1.0

        i10 = 5*(t0**4)
        i11 = 4*(t0**3)
        i12 = 3*(t0**2)
        i13 = 2*t0
        i14 = 1.0
        i15 = 0.0

        i20 = 20*(t0**3)
        i21 = 12*(t0**2)
        i22 = 6*t0
        i23 = 2.0
        i24 = 0.0
        i25 = 0.0

        i30 = (tf**5)
        i31 = (tf**4)
        i32 = (tf**3)
        i33 = (tf**2)
        i34 = tf
        i35 = 1.0

        i40 = 5*(tf**4)
        i41 = 4*(tf**3)
        i42 = 3*(tf**2)
        i43 = 2*tf
        i44 = 1.0
        i45 = 0.0

        i50 = 20*(tf**3)
        i51 = 12*(tf**2)
        i52 = 6*tf
        i53 = 2.0
        i54 = 0.0
        i55 = 0.0

        A = np.array([[i00, i01, i02, i03, i04, i05],
                      [i10, i11, i12, i13, i14, i15],
                      [i20, i21, i22, i23, i24, i25],
                      [i30, i31, i32, i33, i34, i35],
                      [i40, i41, i42, i43, i44, i45],
                      [i50, i51, i52, i53, i54, i55]])

        b = np.array([x0,
                      v0,
                      a0,
                      xf,
                      vf,
                      af])

        x = np.linalg.solve(A,b)
        coefficient_matrix = np.array([[   x[0],    x[1],   x[2],   x[3], x[4], x[5]],
                                       [  0.0, 5*x[0],  4*x[1], 3*x[2], 2*x[3], x[4]],
                                       [  0.0, 0.0, 20*x[0], 12*x[1], 6*x[2], 2*x[3]]])

        row1 = time_array ** 5
        row2 = time_array ** 4
        row3 = time_array ** 3
        row4 = time_array ** 2
        row5 = time_array
        row6 = np.ones(int(size))
        time_matrix = np.array([row1,row2,row3,row4,row5,row6])
        data = np.dot(coefficient_matrix,time_matrix) #thetas are 1st row, dthetas are 2nd row, ddthetas are 3rd row
        # plt.plot(time_array,data[2][:])
        # plt.show()
        return data, time_array

    def Make_joint_profile(self,t=0,v2=0,max=False,final_velocity=False):

        thetas = []
        dthetas = []
        ddthetas = []
        dddthetas = []
        time = []
        relative_time = []
        all_data = []
        all_time = []

        if max:

            if self.point_by_point:
                self.mode = self.Max_joint_decision_making(v_f=v2, velocity_mode=final_velocity)
            else:
                self.mode = self.Max_joint_whole_profile(v_f=v2, velocity_mode=final_velocity)

        else:
            self.mode = self.Time_limited_joint_decision_making(t=t,v2=v2,final_velocity=final_velocity)

        #print(" mode is ", self.mode)
        if self.mode == "None" :
            if self.interpolation:
                return all_data, all_time
            return thetas, dthetas, ddthetas, dddthetas

        elif self.mode == "Linear" :

            d = self.distance
            self.__v_2 = self.v_max
            self.__t_f = d/self.__v_2
            if self.interpolation:
                data, time_array = self.Interpolate(0, self.__t_f, self.p1, self.p2,
                                                    self.__v_2, self.__v_2, 0, 0)
                all_data.append(data)
                all_time.append(time_array)
            else:

                time.append(0)
                time.append(self.__t_f)
                thetas.append(self.p1)
                thetas.append(self.p1 + self.__v_2*self.__t_f)
                dthetas.append(self.__v_2)
                dthetas.append(self.__v_2)
                ddthetas.append(0)
                ddthetas.append(0)
                dddthetas.append(0)
                dddthetas.append(0)

        else:

            time.append(0)
            thetas.append(self.p1)
            dthetas.append(self.__v_1)
            ddthetas.append(self.__a_1)
            if self.__t_a > 0:
                time.append(self.__t_a + time[-1])
                thetas.append(self.__theta_a)
                dthetas.append(self.__v_a)
                ddthetas.append(self.a_max)
            if self.__t_b > 0:
                time.append(self.__t_b + time[-1])
                thetas.append(self.__theta_b+thetas[-1])
                dthetas.append(self.__v_b+dthetas[-1])
                ddthetas.append(ddthetas[-1])
            if self.__t_d > 0:
                time.append(self.__t_d+time[-1])
                thetas.append(self.__theta_d+thetas[-1])
                dthetas.append(self.__v_d+dthetas[-1])
                ddthetas.append(self.__a_final)
            if self.__t_f > 0:
                time.append(self.__t_f + time[-1])
                thetas.append(self.__theta_f+thetas[-1])
                dthetas.append(self.__v_2)
                ddthetas.append(ddthetas[-1])

            if self.interpolation:
                for i in range(len(time)-1):
                    data, time_array = self.Interpolate(time[i], time[i+1], thetas[i], thetas[i+1],
                                                        dthetas[i], dthetas[i+1], ddthetas[i], ddthetas[i+1])
                    all_data.append(data)
                    all_time.append(time_array)

        if self.interpolation:
            return all_data, all_time
        return thetas,dthetas, ddthetas, dddthetas, time

    def Make_profile(self, t=0, v2=0, final_velocity=False):

        joint_thetas = []
        joint_dthetas = []
        joint_ddthetas = []
        joint_dddthetas = []
        time = []
        all_data = []
        all_time = []



        if t == 0:
            if self.interpolation:
                all_data, all_time = self.Make_joint_profile(max=True,v2=v2,final_velocity=final_velocity)
            else:
                joint_thetas, joint_dthetas, joint_ddthetas, joint_dddthetas, time = \
                    self.Make_joint_profile(max=True,v2=v2,final_velocity=final_velocity)
        elif t > 0:
            if self.interpolation:
                all_data, all_time = self.Make_joint_profile(t=t, v2=v2, max=False, final_velocity=final_velocity)
            else:
                joint_thetas, joint_dthetas, joint_ddthetas, joint_dddthetas, time = \
                    self.Make_joint_profile(t=t, v2=v2, max=False, final_velocity=final_velocity)
        else:
            ##print("ERROR:: time < 0 for a normal joint")
            self.mode = "None"

        if self.mode == "None" :
            if self.interpolation:
                return all_data, all_time
            return joint_thetas, joint_dthetas, joint_ddthetas, joint_dddthetas, self.__t_f
        if self.interpolation:
            size = 0
            for i in range(len(all_data)):
                size = size + all_data[i].shape[1]
            all_data_array = np.zeros((3,size))
            all_time_array = np.zeros((size,))
            np.concatenate(all_data, axis=1, out=all_data_array)
            np.concatenate(all_time, axis=0, out=all_time_array)
            if self.flip_data:
                all_data_array[0] = -all_data_array[0] + 2*self.p1 + self.p2 - self.p1
                all_data_array[1] = -all_data_array[1]
                all_data_array[2] = -all_data_array[2]
            return all_data_array, all_time_array

        if self.flip_data :

            thetas = np.array(joint_thetas)
            thetas = -thetas + 2*self.p1 + self.p2 - self.p1
            dthetas = np.array(joint_dthetas)
            ddthetas = np.array(joint_ddthetas)
            ddthetas = -1*ddthetas
            dddthetas = np.array(joint_dddthetas)
            dthetas = -1*dthetas
            dddthetas = -1 * dddthetas
            time_array = np.array(time)

        else :

            time_array = np.array(time)
            thetas = np.array(joint_thetas)
            dthetas = np.array(joint_dthetas)
            ddthetas = np.array(joint_ddthetas)
            dddthetas = np.array(joint_dddthetas)

        return thetas, dthetas, ddthetas, dddthetas, time_array

    def Make_path_profile(self,path):

        all_data_list = []
        all_time_list = []
        all_data = np.array([])
        all_time = np.array([])
        total_time = 0

        prev_v = 0
        prev_a = 0

        a = STOPP(path[0], path[((path.size - 1) / 2)-1], self.__j_max, self.__a_max, self.__v_max, self.time_step, self.__v_1,
                  self.__a_1, path[-1],
                  interpolation=False,point_by_point=False)

        a.Max_joint_whole_profile()
        self.__v_max = a.v_max

        if a.mode == 'None':
            return [],[]

        for i in range((path.size - 1) / 2):

            a = STOPP(path[i], path[i + 1], self.__j_max, self.__a_max, self.__v_max, self.time_step, prev_v,
                      prev_a, path[-1],
                      interpolation=self.interpolation)
            # start_time = time.time()
            # a.Get_total_time()
            # #print("time = ", time.time() - start_time)
            # #print("decision_time = ",a.total_time)

            if self.interpolation:
                traj_data, traj_time = a.Make_profile()
            else:
                max_joint_thetas, max_joint_dthetas, max_joint_ddthetas, _, t_f = \
                    a.Make_profile()
                traj_data = np.array([max_joint_thetas, max_joint_dthetas, max_joint_ddthetas])
                traj_time = t_f

            if a.flip_data:
                prev_v = -traj_data[1][-1]
                prev_a = -traj_data[2][-1]
            else:
                prev_v = traj_data[1][-1]
                prev_a = traj_data[2][-1]

            if a.mode == 'None':
                break

            if i == 0:
                print("first_point = {}".format(path[i]))
                all_time_list.append(traj_time + total_time)
                all_data_list.append(traj_data)
            else:
                all_time_list.append(traj_time[1:] + total_time)
                all_data_list.append(traj_data[:, 1:])

            total_time = total_time + traj_time[-1]
            # plt.plot(all_time, all_data[2])
            # plt.show()


        all_data = np.column_stack(all_data_list)
        all_time = np.concatenate(all_time_list, axis=0)

        return all_data, all_time

    def Logging(self,file_path,all_data,all_time=0,interpolate = False,joint_number=6):

            all_time_relative_temp = np.diff(all_time)
            all_time_relative = np.zeros(all_time_relative_temp.size + 1,)
            all_time_relative[1:] = all_time_relative_temp[0:]
            qs = open(file_path+"qs.txt", "w+")
            for i in range(all_data[0][0].size):
                for j_num in range(joint_number):
                        qs.write("%.5f\r\n" % (all_data[j_num][0][i]*(180.0/pi)))

            qs.close()
            qds = open(file_path+"qds.txt", "w+")
            for i in range(all_data[0][0].size):
                for j_num in range(joint_number):
                    qds.write("%.5f\r\n" % (all_data[j_num][1][i]*(180.0/pi)))

            qds.close()
            qdds = open(file_path+"qdds.txt", "w+")
            for i in range(all_data[0][0].size):
                for j_num in range(joint_number):
                    qdds.write("%.5f\r\n" % (all_data[j_num][2][i]*(180.0/pi)))

            qdds.close()
            if not interpolate:
                ts = open(file_path+"ts.txt", "w+")
                for i in range(all_time_relative.size):
                    ts.write("%.5f\r\n" % (all_time_relative[i]))
                ts.close()


