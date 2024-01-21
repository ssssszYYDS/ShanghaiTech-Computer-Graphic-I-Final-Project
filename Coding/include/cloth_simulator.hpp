#pragma once

#include <iostream>

#include "Eigen/Core"
#include "Eigen/Dense"

#include "cloth.hpp"
#include "define.hpp"

class RectClothSimulator {
private:
    struct MassParticle {
        glm::vec3 position;
        std::vector<unsigned int> connectedSpringStartIndices;
        std::vector<unsigned int> connectedSpringEndIndices;

        // TODO: define other particle properties here
        glm::vec3 velocity;
        float mass;
    };

    struct Spring {
        unsigned int fromMassIndex;
        unsigned int toMassIndex;

        // TODO: define other spring properties here
        float stiffness;
        float restLength;
    };

    RectCloth *cloth;
    std::vector<MassParticle> particles;
    std::vector<Spring> springs;

    // Simulation parameters
    glm::vec3 gravity;
    float airResistanceCoefficient; // Per-particle

    // bonus
    float windCoefficient;
    float windStrength;
    glm::vec3 windDirection;

    MassParticle *scratchPoint = nullptr;
    float scratchDistance = INFINITY;

    // Project
    const Eigen::Matrix3f I3 = Eigen::Matrix3f::Identity();
    Eigen::MatrixXf M;
    Eigen::MatrixXf L;
    Eigen::MatrixXf J;
    Eigen::MatrixXf h2J;
    Eigen::MatrixXf M_p_h2L;
    Eigen::LDLT<Eigen::MatrixXf> ldlt;

    Eigen::MatrixXf q_n;
    Eigen::MatrixXf q_n_1;

    Eigen::MatrixXf h2f_ext;

    int maxIter = 10;
    float timeStep = 0.00001f;
    float totalMass = 1.0f;

public:
    RectClothSimulator(
        RectCloth *cloth,
        float totalMass,
        float stiffnessReference,
        float airResistanceCoefficient,
        const glm::vec3 &gravity,
        const float timeStep);

    // bonus
    RectClothSimulator(
        RectCloth *cloth,
        float totalMass,
        float stiffnessReference,
        float airResistanceCoefficient,
        const glm::vec3 &gravity, float windCoefficient, glm::vec3 windDirection,
        const float timeStep);

    // project
    RectClothSimulator(
        RectCloth *cloth,
        float totalMass,
        float stiffnessReference,
        float airResistanceCoefficient,
        const glm::vec3 &gravity,
        const float timeStep,
        const int maxIter);

    ~RectClothSimulator() = default;

    void step();
    void step2();

    // bonus
    void updateScratchPoint(glm::vec3 ori, glm::vec3 dir, bool update);

private:
    void createMassParticles(float totalMass);
    void createSprings(float stiffnessReference);
    void updateCloth();

    // project
    void initBasicValues();

    static glm::vec3 calcNormal(const glm::vec3 &v1, const glm::vec3 &v2, const glm::vec3 &v3) { return glm::normalize(glm::cross(v2 - v1, v3 - v1)); };

    static float calcArea(const glm::vec3 &v1, const glm::vec3 &v2, const glm::vec3 &v3) {
        return glm::length(glm::cross(v2 - v1, v3 - v1)) / 2.0f;
    }

    // project
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> kroneckerProduct(const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &A, const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &B);

    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> getMassMatrix();

    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> getLaplacianMatrix();

    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> getJacobianMatrix();

    Eigen::Matrix<float, Eigen::Dynamic, 1> getExternalForceVector();

    Eigen::Matrix<float, Eigen::Dynamic, 1> getBVector(Eigen::Matrix<float, Eigen::Dynamic, 1> y);

    Eigen::Matrix<float, Eigen::Dynamic, 1> solve(Eigen::Matrix<float, Eigen::Dynamic, 1> y, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> M_p_h2L, int maxIter);

    void applyConstraints();

    // float E(std::vector<MassParticle>& particles, std::vector<Spring>& springs);

#if TEST
public:
    void test() {
        // auto A = getIncidenceVector(0);
        // auto A_T = A.transpose();
        // auto A_A_T = A * A_T;

        // std::cout << "shape of A: " << A.rows() << " " << A.cols() << std::endl;
        // std::cout << "shape of A_T: " << A_T.rows() << " " << A_T.cols() << std::endl;
        // std::cout << "shape of A_A_T: " << A_A_T.rows() << " " << A_A_T.cols() << std::endl;
    }
#endif
};