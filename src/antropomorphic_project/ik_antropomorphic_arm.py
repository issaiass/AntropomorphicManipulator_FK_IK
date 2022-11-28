#!/usr/bin/env python3

from math import atan2, pi, sin, cos, pow, sqrt



from dataclasses import dataclass
@dataclass
class EndEffectorWorkingSpace:
    # Pos of The P2, which is the one we resolved for
    Pee_x: float
    Pee_y: float
    Pee_z: float

class ComputeIk():

    def __init__(self, DH_parameters):
        
        # DH parameters
        self.DH_parameters_ = DH_parameters

    def get_dh_param(self, name):

        if name in self.DH_parameters_:
            return self.DH_parameters_[name]
        else:
            assert False, "Asked for Non existen param DH name ="+str(name)

    def compute_ik(self, end_effector_pose, theta_2_config = "plus", theta_3_config = "plus"):
        
        # Initialization
        Pee_x = end_effector_pose.Pee_x
        Pee_y = end_effector_pose.Pee_y
        Pee_z = end_effector_pose.Pee_z


        # We get all the DH parameters
        r2 = self.get_dh_param("r2")
        r3 = self.get_dh_param("r3")

        print("Input Data===== theta_2_config = "+str(theta_2_config))
        print("Input Data===== theta_2_config = "+str(theta_3_config))        
        print("Pee_x = "+str(Pee_x))
        print("Pee_y = "+str(Pee_y))
        print("Pee_z = "+str(Pee_z))
        print("r2 = "+str(r2))
        print("r3 = "+str(r3))

        # We declare all the equations for theta1, theta2, theta3 and auxiliary
        #########################################################################
        # theta_2
        possible_solution = True
        if Pee_x==0 and Pee_y==0:
           print("End effector X and Y must be greather than 0")
           possible_solution = False
        if r2==0 or r3==0:
           print("r2 or r3 must be positive both and non-zero")
           possible_solution = False
        D = (Pee_x**2 + Pee_y**2 + Pee_z**2 - r2 - r3)/(2*r2*r3)
        print(D)


        theta_array = []

        if possible_solution:
            #########################################################################


            #########################################################################
            # theta3
            # We have to decide which solution we want
            if theta_3_config == "plus":
                # Positive
                numerator_1 = sqrt(1 - pow(D,2))
            else:
                numerator_1 = -1.0 * sqrt(1-pow(D,2))

            theta_3 = atan2(numerator_1, D)
            #########################################################################


            #########################################################################
            # theta2
            s3 = sin(theta_3)
            c3 = cos(theta_3)

            numerator_2a = Pee_z
            denominator_2a = sqrt(pow(Pee_x, 2) + pow(Pee_y, 2))
            numerator_2b = r3*s3
            if theta_2_config == "plus":
                denominator_2b = r2 + r3*c3
            else:
                denominator_2b = r2 + r3*(-c3)


            theta_beta = atan2(numerator_2a, denominator_2a)
            theta_alpha = atan2(numerator_2b, denominator_2b)

            theta_2 = theta_beta - theta_alpha

            #########################################################################


            #########################################################################
            # theta1
            # 
            theta_1 = atan2(Pee_y, Pee_x)
            #########################################################################

            theta_array = [theta_1, theta_2, theta_3]
        if theta_2 <= -pi/4 or theta_2 >= 3*pi/4:
            possible_solution = False
            print(f"theta_2 NOT POSSIBLE, MIN={-pi/4}, theta_2={theta_2}, MAX={3*pi/4}")
        if theta_3 <= -3*pi/4 or theta_3 >= 3*pi/4:
            print(f"theta_3 NOT POSSIBLE, MIN={-3*pi/4}, theta_2={theta_2}, MAX={3*pi/4}")        
            possible_solution = False
        return theta_array, possible_solution



def calculate_ik(Pee_x, Pee_y, Pee_z, DH_parameters, theta_2_config, theta_3_config):

    ik = ComputeIk(DH_parameters = DH_parameters)
    end_effector_pose = EndEffectorWorkingSpace(Pee_x = Pee_x,
                                                Pee_y = Pee_y,
                                                Pee_z = Pee_z)

    thetas, possible_solution = ik.compute_ik(end_effector_pose=end_effector_pose, theta_2_config=theta_2_config, theta_3_config=theta_3_config)

    print("Angles thetas solved ="+str(thetas))
    print("possible_solution = "+str(possible_solution))
    print()

    return thetas, possible_solution

if __name__ == '__main__':
    

    r2 = 1.0
    r3 = 1.0

    # theta_i here are valriables of the joints
    # We only fill the ones we use in the equations, the others were already 
    # replaced in the Homogeneous matrix
    DH_parameters={"r2":r2, "r3":r3}

    Pee_x = 0.5
    Pee_y = 0.6
    Pee_z = 0.7

    calculate_ik(Pee_x=Pee_x, Pee_y=Pee_y, Pee_z=Pee_z, DH_parameters=DH_parameters, theta_2_config = "plus", theta_3_config = "plus")
    calculate_ik(Pee_x=Pee_x, Pee_y=Pee_y, Pee_z=Pee_z, DH_parameters=DH_parameters, theta_2_config = "plus", theta_3_config = "minus")
    calculate_ik(Pee_x=Pee_x, Pee_y=Pee_y, Pee_z=Pee_z, DH_parameters=DH_parameters, theta_2_config = "minus", theta_3_config = "plus")
    calculate_ik(Pee_x=Pee_x, Pee_y=Pee_y, Pee_z=Pee_z, DH_parameters=DH_parameters, theta_2_config = "minus", theta_3_config = "minus")