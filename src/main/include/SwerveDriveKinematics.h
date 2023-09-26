#include <iostream>
#include <frc/EigenCore.h>
#include "Translation2d.h"
#include "Rotation2d.h"
#include <vector>
#include "Eigen/QR"
#include "ChassisSpeeds.h"
#include <cmath>
#include "SwerveModuleState.h"

class SwerveDriveKinematics {
    private: 
        const static int m_numModules = 4;
        frc::Matrixd<m_numModules * 2, 2> testMatrix;
        frc::Matrixd<m_numModules * 2, 3> m_inverseKinematics;
        frc::Matrixd<3, m_numModules * 2> m_forwardsKinematics;
        Rotation2d m_rotations[4];
        frc::Matrixd<3, 1> chassisSpeedsVector;

    public:

        SwerveDriveKinematics(std::vector<Translation2d> wheelsPos) {
            //m_numModules = wheelsPos.size();
            std::vector<Translation2d> m_modules = wheelsPos;
            

            for (int i = 0; i < m_numModules; i++) {
                m_inverseKinematics.block<2, 3>(i * 2, 0) = frc::Matrixd<2, 3>{
                {1, 0, float(-m_modules[i].y())},
                {0, 1, float(m_modules[i].x())}
                }; 
                m_rotations[i] = Rotation2d(atan2(m_modules[i].y(), m_modules[i].x()));
            }

            m_forwardsKinematics = m_inverseKinematics.completeOrthogonalDecomposition().pseudoInverse();
            

        }

        std::vector<SwerveModuleState> toSwerveStates(ChassisSpeeds desiredSpeeds) {
            // Its a vector but I used a matrix sorry ik theres a vector class
            chassisSpeedsVector = frc::Matrixd<3, 1> {{float(desiredSpeeds.vxMetersPerSecond)},
                                                        {float(desiredSpeeds.vyMetersPerSecond)}, 
                                                        {float(desiredSpeeds.omegaRadiansPerSecond)}};

            //std::cout << chassisSpeedsVector;
            // I used Eigen::Dynamic even tho I didn't import Eigen on this file so thats weird
            // Make sure to add the Eigen/Dynamic include in the future
            frc::Matrixd<Eigen::Dynamic, Eigen::Dynamic> moduleStatesMatrix = m_inverseKinematics * chassisSpeedsVector; // Matrix Multiplication
            std::vector<SwerveModuleState> moduleStates;

            for (int i = 0; i < m_numModules; i++) {
                    double x = moduleStatesMatrix(i * 2, 0);
                    double y = moduleStatesMatrix(i * 2 + 1, 0);

                    double speed = hypot(x, y);
                    Rotation2d angle = Rotation2d(atan2(y, x));
                    moduleStates.push_back(SwerveModuleState(speed, angle));
                    //std::cout << "\n" << speed << ", " << angle.getDegrees();
            }

            return moduleStates;

            
        }    
};
