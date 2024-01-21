#include <iostream>
#include <omp.h>

#include "Eigen/Cholesky"
#include "Eigen/Dense"

#include "cloth_renderer.hpp"
#include "cloth_simulator.hpp"
#include <unordered_set>

RectClothSimulator::
    RectClothSimulator(
        RectCloth *cloth,
        float totalMass,
        float stiffnessReference,
        float airResistanceCoefficient,
        const glm::vec3 &gravity,
        const float timeStep) : cloth(cloth), airResistanceCoefficient(airResistanceCoefficient), gravity(gravity), windCoefficient(0.0f), windDirection(glm::vec3(0.0f, 0.0f, 1.0f)), timeStep(timeStep) {
    // Initialize particles, then springs according to the given cloth
    createMassParticles(totalMass);
    createSprings(stiffnessReference);
}

RectClothSimulator::
    RectClothSimulator(
        RectCloth *cloth,
        float totalMass,
        float stiffnessReference,
        float airResistanceCoefficient,
        const glm::vec3 &gravity, float windCoefficient, glm::vec3 windDirection,
        const float timeStep) : cloth(cloth), airResistanceCoefficient(airResistanceCoefficient), gravity(gravity), windCoefficient(windCoefficient), windDirection(windDirection / glm::length(windDirection)), timeStep(timeStep) {
    // Initialize particles, then springs according to the given cloth
    createMassParticles(totalMass);
    createSprings(stiffnessReference);
}

// project
RectClothSimulator::
    RectClothSimulator(
        RectCloth *cloth,
        float totalMass,
        float stiffnessReference,
        float airResistanceCoefficient,
        const glm::vec3 &gravity,
        const float timeStep,
        const int maxIter) : cloth(cloth), airResistanceCoefficient(airResistanceCoefficient), gravity(gravity), windCoefficient(0.0f), windDirection(glm::vec3(0.0f, 0.0f, 1.0f)), timeStep(timeStep), maxIter(maxIter) {
    // Initialize particles, then springs according to the given cloth
    createMassParticles(totalMass);
    createSprings(stiffnessReference);
    initBasicValues();
}

void RectClothSimulator::
    createMassParticles(float totalMass) {
    // Create mass particles based on given cloth.
    particles.resize(cloth->nw * cloth->nh);
    for (unsigned int ih = 0; ih < cloth->nh; ih++) {
        for (unsigned int iw = 0; iw < cloth->nw; iw++) {
            MassParticle particle;
            particle.position = cloth->getPosition(iw, ih);

            // TODO: Initialize other mass properties.
            //  Use 'cloth->...' to access cloth properties.

            particle.velocity = glm::vec3(0.0f);
            particle.mass = totalMass / (cloth->nw * cloth->nh);

            if (iw > 0) {
                particle.connectedSpringStartIndices.push_back(cloth->idxFromCoord(iw, ih));
                particle.connectedSpringEndIndices.push_back(cloth->idxFromCoord(iw - 1, ih));
                if (ih > 0) {
                    particle.connectedSpringStartIndices.push_back(cloth->idxFromCoord(iw, ih));
                    particle.connectedSpringEndIndices.push_back(cloth->idxFromCoord(iw - 1, ih - 1));
                }
                if (ih < cloth->nh - 1) {
                    particle.connectedSpringStartIndices.push_back(cloth->idxFromCoord(iw, ih));
                    particle.connectedSpringEndIndices.push_back(cloth->idxFromCoord(iw - 1, ih + 1));
                }
            }
            if (iw < cloth->nw - 1) {
                particle.connectedSpringStartIndices.push_back(cloth->idxFromCoord(iw, ih));
                particle.connectedSpringEndIndices.push_back(cloth->idxFromCoord(iw + 1, ih));
                if (ih > 0) {
                    particle.connectedSpringStartIndices.push_back(cloth->idxFromCoord(iw, ih));
                    particle.connectedSpringEndIndices.push_back(cloth->idxFromCoord(iw + 1, ih - 1));
                }
                if (ih < cloth->nh - 1) {
                    particle.connectedSpringStartIndices.push_back(cloth->idxFromCoord(iw, ih));
                    particle.connectedSpringEndIndices.push_back(cloth->idxFromCoord(iw + 1, ih + 1));
                }
            }
            if (ih > 0) {
                particle.connectedSpringStartIndices.push_back(cloth->idxFromCoord(iw, ih));
                particle.connectedSpringEndIndices.push_back(cloth->idxFromCoord(iw, ih - 1));
            }
            if (ih < cloth->nh - 1) {
                particle.connectedSpringStartIndices.push_back(cloth->idxFromCoord(iw, ih));
                particle.connectedSpringEndIndices.push_back(cloth->idxFromCoord(iw, ih + 1));
            }

            particles[cloth->idxFromCoord(iw, ih)] = particle;
        }
    }
    this->totalMass = totalMass;
}

#if SPRING_DECREASE
struct pair_hash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2> &p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        return h1 ^ h2;
    }
};
#endif

void RectClothSimulator::
    createSprings(float stiffnessReference) {
    // First clear all springs
    springs.clear();

    // TODO: Create springs connecting mass particles.
    //  You may find 'cloth->idxFromCoord(...)' useful.
    //  You can store springs into the member variable 'springs' which is a std::vector.
    //  You may want to modify mass particles too.

    for (MassParticle &particle : particles) {
        for (unsigned int i = 0; i < particle.connectedSpringStartIndices.size(); ++i) {
            Spring spring;
            spring.fromMassIndex = particle.connectedSpringStartIndices[i];
            spring.toMassIndex = particle.connectedSpringEndIndices[i];
            spring.stiffness = stiffnessReference;
            spring.restLength = glm::length(particle.position - particles[spring.toMassIndex].position);
            springs.push_back(spring);
        }
    }

#if SPRING_DECREASE
    std::unordered_set<std::pair<int, int>, pair_hash> springIndices;
    std::vector<Spring> updatedSprings;

    for (const Spring &spring : springs) {
        std::pair<int, int> springIndex1 = {spring.fromMassIndex, spring.toMassIndex};
        std::pair<int, int> springIndex2 = {spring.toMassIndex, spring.fromMassIndex};
        if (springIndices.count(springIndex1) == 0 && springIndices.count(springIndex2) == 0) {
            springIndices.insert(springIndex1);
            springIndices.insert(springIndex2);
            updatedSprings.push_back(spring);
        }
    }

    springs = updatedSprings;
#endif
}

void RectClothSimulator::
    step() {
    // TODO: Simulate one step based on given time step.
    //  Step 1: Update particle positions
    //  Step 2: Update springs
    //  Step 3: Apply constraints
    //  Hint: See cloth_simulator.hpp to check for member variables you need.
    //  Hint: You may use 'cloth->getInitialPosition(...)' for constraints.

    // Step 1: Update particle positions
    for (MassParticle &particle : particles) {
        particle.position += particle.velocity * timeStep;
    }

    // Step 2: Update springs
    for (Spring &spring : springs) {
        MassParticle &fromParticle = particles[spring.fromMassIndex];
        MassParticle &toParticle = particles[spring.toMassIndex];

        glm::vec3 springVector = toParticle.position - fromParticle.position;
        float springLength = glm::length(springVector);
        glm::vec3 springDirection = springVector / springLength;

        float springForce = spring.stiffness * (springLength - spring.restLength);
        glm::vec3 springForceVector = springForce * springDirection;

        fromParticle.velocity += springForceVector / fromParticle.mass * timeStep;
    }

#if BONUS_WIND
    // bonus wind
    std::vector<glm::vec3>
        particlesNormals = std::vector<glm::vec3>(cloth->nw * cloth->nh, glm::vec3(0.0f));
    for (unsigned int ih = 0; ih < cloth->nh - 1; ++ih) {
        for (unsigned int iw = 0; iw < cloth->nw - 1; ++iw) {
            MassParticle &particle = particles[cloth->idxFromCoord(iw, ih)];

            unsigned int leftDownIdx = this->cloth->idxFromCoord(iw, ih);
            unsigned int rightDownIdx = this->cloth->idxFromCoord(iw + 1, ih);
            unsigned int leftUpIdx = this->cloth->idxFromCoord(iw, ih + 1);
            unsigned int rightUpIdx = this->cloth->idxFromCoord(iw + 1, ih + 1);

            glm::vec3 lowerNormal = calcNormal(
                particles[leftDownIdx].position,
                particles[rightDownIdx].position,
                particles[rightUpIdx].position);
            glm::vec3 upperNormal = calcNormal(
                particles[leftDownIdx].position,
                particles[rightUpIdx].position,
                particles[leftUpIdx].position);
            particlesNormals[leftDownIdx] += lowerNormal;
            particlesNormals[rightDownIdx] += lowerNormal;
            particlesNormals[rightUpIdx] += lowerNormal;
            particlesNormals[leftDownIdx] += upperNormal;
            particlesNormals[rightUpIdx] += upperNormal;
            particlesNormals[leftUpIdx] += upperNormal;
        }
    }
#endif

    // Step 3.5: Apply gravity and air resistance, and bonus wind
    for (unsigned int i = 0; i < particles.size(); ++i) {
        MassParticle &particle = particles[i];
        particle.velocity += gravity * timeStep;
#if AIR_RESIS
        particle.velocity -= airResistanceCoefficient * particle.velocity * glm::length(particle.velocity) / particle.mass * timeStep;
#endif

#if BONUS_WIND
        glm::vec3 lowerNormal = particlesNormals[i] / glm::length(particlesNormals[i]);
        glm::vec3 upperNormal = particlesNormals[i] / glm::length(particlesNormals[i]);
        particle.velocity += (std::abs(glm::dot(lowerNormal, windDirection)) + std::abs(glm::dot(upperNormal, windDirection))) * (0.5f * cloth->dx * cloth->dx / particle.mass) * (windCoefficient * windDirection) * timeStep;
#endif
    }

    // Step 4: Apply constraints
    // the constraints are fixing the left-up and right-up corners of the cloth
    MassParticle &left_up = particles[cloth->idxFromCoord(0, 0)];
    MassParticle &right_up = particles[cloth->idxFromCoord(cloth->nw - 1, 0)];
    left_up.position = cloth->getInitialPosition(0, 0);
    right_up.position = cloth->getInitialPosition(cloth->nw - 1, 0);
    left_up.velocity = glm::vec3(0.0f);
    right_up.velocity = glm::vec3(0.0f);

    // Finally update cloth data
    updateCloth();
}

void RectClothSimulator::
    step2() {
    applyConstraints();

    auto y = 2 * q_n - q_n_1;
    // std::cout << "b_size: " << b.rows() << " " << b.cols() << std::endl;

    auto q_n_plus_1 = solve(y, M_p_h2L, maxIter);

    q_n_1 = q_n;
    q_n = q_n_plus_1;

    for (int i = 0; i < particles.size(); ++i) {
        particles[i].position = glm::vec3(q_n(3 * i, 0), q_n(3 * i + 1, 0), q_n(3 * i + 2, 0));
        particles[i].velocity = glm::vec3(q_n(3 * i, 0), q_n(3 * i + 1, 0), q_n(3 * i + 2, 0)) - glm::vec3(q_n_1(3 * i, 0), q_n_1(3 * i + 1, 0), q_n_1(3 * i + 2, 0));
    }

    // std::cout << "particles[5].position: " << particles[5].position.x << " " << particles[5].position.y << " " << particles[5].position.z << std::endl;
    // std::cout << "delta: " << (q_n_plus_1 - q_n).norm() << std::endl;

    // Finally update cloth data
    updateCloth();
}

void RectClothSimulator::updateCloth() {
    for (unsigned int i = 0u; i < cloth->nw * cloth->nh; ++i) {
        cloth->setPosition(i, particles[i].position);
    }
}

void RectClothSimulator::updateScratchPoint(glm::vec3 ori, glm::vec3 dir, bool update) {
    if (update && scratchPoint != nullptr) {
#if !PROJECT // bonus
        scratchPoint->position = ori + scratchDistance * dir;
        scratchPoint->velocity = glm::vec3(0.0f);
#else // project
        auto newPosition = ori + scratchDistance * dir;
        Eigen::Vector3f newPositionEigen(newPosition.x, newPosition.y, newPosition.z);
        int scratch_index = scratchPoint - &particles[0];
        q_n.block<3, 1>(3 * scratch_index, 0) = newPositionEigen;
        q_n_1.block<3, 1>(3 * scratch_index, 0) = newPositionEigen;
#endif
    }
    if (update && scratchPoint == nullptr) {
        float dist = INFINITY;
        for (MassParticle &particle : particles) {
            glm::vec3 diffVec = particle.position - ori;
            glm::vec3 distanceVec = diffVec - glm::dot(diffVec, dir) * dir;
            float curDist = glm::length(distanceVec);
            if (curDist < dist) {
                dist = curDist;
                scratchPoint = &particle;
                scratchDistance = glm::dot(diffVec, dir);
            }
        }
    }
    if (!update && scratchPoint != nullptr) {
        scratchPoint = nullptr;
        scratchDistance = INFINITY;
    }
    // std::cout << "Scratch point " << scratchPoint << std::endl;
    // std::cout << "Distance " << scratchDistance << std::endl;
}

// project
void RectClothSimulator::initBasicValues() {
    std::cout << "particle number: " << particles.size() << std::endl;
    std::cout << "spring number: " << springs.size() << std::endl;

    auto stratTime = omp_get_wtime();
    M = getMassMatrix();
    std::cout << "M_size: " << M.rows() << " " << M.cols() << std::endl;
    std::cout << "Inital mass matrix time: " << (omp_get_wtime() - stratTime) * 1000 << "ms" << std::endl
              << std::endl;

    stratTime = omp_get_wtime();
    L = getLaplacianMatrix();
    std::cout << "L_size: " << L.rows() << " " << L.cols() << std::endl;
    std::cout << "Inital laplacian matrix time: " << (omp_get_wtime() - stratTime) * 1000 << "ms" << std::endl
              << std::endl;

    M_p_h2L = M + timeStep * timeStep * L;
    std::cout << "M_p_h2L_size: " << M_p_h2L.rows() << " " << M_p_h2L.cols() << std::endl;

    stratTime = omp_get_wtime();
    ldlt = M_p_h2L.ldlt();
    std::cout << "ldlt_size: " << ldlt.rows() << " " << ldlt.cols() << std::endl;
    std::cout << "ldlt time: " << (omp_get_wtime() - stratTime) * 1000 << "ms" << std::endl
              << std::endl;

    stratTime = omp_get_wtime();
    J = getJacobianMatrix();
    std::cout << "J_size: " << J.rows() << " " << J.cols() << std::endl;
    std::cout << "Inital jacobian matrix time: " << (omp_get_wtime() - stratTime) * 1000 << "ms" << std::endl
              << std::endl;

    h2J = timeStep * timeStep * J;
    std::cout << "h2J_size: " << h2J.rows() << " " << h2J.cols() << std::endl;

    q_n = Eigen::MatrixXf::Zero(3 * particles.size(), 1);
    q_n_1 = Eigen::MatrixXf::Zero(3 * particles.size(), 1);
    for (int i = 0; i < particles.size(); ++i) {
        q_n.block<3, 1>(3 * i, 0) = Eigen::Vector3f(particles[i].position.x, particles[i].position.y, particles[i].position.z);
        q_n_1.block<3, 1>(3 * i, 0) = Eigen::Vector3f(particles[i].position.x, particles[i].position.y, particles[i].position.z);
    }
    std::cout << "q_n_size: " << q_n.rows() << " " << q_n.cols() << std::endl;
    std::cout << "q_n_1_size: " << q_n_1.rows() << " " << q_n_1.cols() << std::endl;

    h2f_ext = getExternalForceVector();
    std::cout << "h2f_ext_size: " << h2f_ext.rows() << " " << h2f_ext.cols() << std::endl;
}

Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> RectClothSimulator::kroneckerProduct(const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &A, const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &B) {
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> result(A.rows() * B.rows(), A.cols() * B.cols());

    for (int i = 0; i < A.rows(); ++i) {
        for (int j = 0; j < A.cols(); ++j) {
            result.block(i * B.rows(), j * B.cols(), B.rows(), B.cols()) = A(i, j) * B;
        }
    }
    return result;
}

Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> RectClothSimulator::getMassMatrix() {
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> massMatrix(3 * particles.size(), 3 * particles.size());
    massMatrix.setZero();
    for (int i = 0; i < particles.size(); ++i) {
        massMatrix.block<3, 3>(3 * i, 3 * i) = particles[i].mass * Eigen::Matrix3f::Identity();
    }
    return massMatrix;
}

Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> RectClothSimulator::getLaplacianMatrix() {
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> laplacianMatrix_left(particles.size(), particles.size());
    laplacianMatrix_left.setZero();

    // #pragma omp parallel for schedule(static) private(laplacianMatrix_left)
    for (int i = 0; i < springs.size(); ++i) {
        laplacianMatrix_left(springs[i].fromMassIndex, springs[i].fromMassIndex) += springs[i].stiffness;
        laplacianMatrix_left(springs[i].fromMassIndex, springs[i].toMassIndex) -= springs[i].stiffness;
        laplacianMatrix_left(springs[i].toMassIndex, springs[i].fromMassIndex) -= springs[i].stiffness;
        laplacianMatrix_left(springs[i].toMassIndex, springs[i].toMassIndex) += springs[i].stiffness;
    }

    return kroneckerProduct(laplacianMatrix_left, I3);
}

Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> RectClothSimulator::getJacobianMatrix() {
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> jacobianMatrix_left(particles.size(), springs.size());
    jacobianMatrix_left.setZero();

    // #pragma omp parallel for schedule(static) private(jacobianMatrix_left)
    for (int i = 0; i < springs.size(); ++i) {
        jacobianMatrix_left(springs[i].fromMassIndex, i) += springs[i].stiffness;
        jacobianMatrix_left(springs[i].toMassIndex, i) -= springs[i].stiffness;
    }

    return kroneckerProduct(jacobianMatrix_left, I3);
}

Eigen::Matrix<float, Eigen::Dynamic, 1> RectClothSimulator::getBVector(Eigen::Matrix<float, Eigen::Dynamic, 1> y) {
    // y = 2q_n - q_{n-1}
    return -M * y + getExternalForceVector();
}

Eigen::Matrix<float, Eigen::Dynamic, 1> RectClothSimulator::getExternalForceVector() {
    Eigen::Matrix<float, Eigen::Dynamic, 1> externalForceVector(3 * particles.size());
    externalForceVector.setZero();
    for (int i = 0; i < particles.size(); ++i) {
        auto G_i = particles[i].mass * gravity;
        auto V_i_glm = particles[i].velocity;
        auto V_i = Eigen::Vector3f(V_i_glm.x, V_i_glm.y, V_i_glm.z);
        externalForceVector.block<3, 1>(3 * i, 0) += Eigen::Vector3f(G_i.x, G_i.y, G_i.z);
#if AIR_RESIS
        externalForceVector.block<3, 1>(3 * i, 0) -= airResistanceCoefficient * V_i * V_i.norm();
#endif
    }
    return timeStep * timeStep * -externalForceVector;
}

/*
    solve for the linear system:
    min_{x \in R^{3m}, d \in U} f(x, d)
    f(x, d) = \frac{1}{2} x^T M_p_h2L x - h^2 x^T J d + x^T b
    where U = {(d_1, ..., d_s) \in R^{2s} | ||d_i|| = restLength_i, i = 1, ..., s}

    => \frac{\partial f}{\partial x} = M_p_h2L x - h^2 J d + b = 0
    => M_p_h2L x = h^2 J d - b
    => x' = M_p_h2L^{-1} (h^2 J d - b)

    d = r (p12 / ||p12||)

*/
Eigen::Matrix<float, Eigen::Dynamic, 1> RectClothSimulator::solve(Eigen::Matrix<float, Eigen::Dynamic, 1> y, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> M_p_h2L, int maxIter) {
    // Perform block coordinate descent
    // Starting with an initial guess for x (we use y)
    // Fix x and compute the optimal d (local step)
    // Fix d and compute the optimal x (global step)
    // Repeat until a maximal number of iterations is reached

    auto stratTime_0 = omp_get_wtime();
    auto b = getBVector(y);
    Eigen::Matrix<float, Eigen::Dynamic, 1> x = y;
    Eigen::Matrix<float, Eigen::Dynamic, 1> d = Eigen::Matrix<float, Eigen::Dynamic, 1>::Zero(3 * springs.size(), 1);

    for (int iter = 1; iter <= maxIter; iter++) {
        // std::cout << "iter: " << iter << std::endl;
        // Local step:
        // Compute optimal d by projecting the springs to their rest lengths
        // No need for Singular Value Decompositions, only vector normalizations
        // d_i = r_i (p12_i / ||p12_i||)
        for (int i = 0; i < springs.size(); ++i) {
            int a = springs[i].fromMassIndex, b = springs[i].toMassIndex;
            Eigen::Vector3f p12 = x.block<3, 1>(3 * a, 0) - x.block<3, 1>(3 * b, 0);
            d.block<3, 1>(3 * i, 0) = springs[i].restLength * p12 / p12.norm();
        }

        // Global step:
        // Fix d and solve a convex quadratic minimization problem for optimal x
        // The system matrix M + h^2L is symmetric positive definite
        // Pre-compute its sparse Cholesky factorization for fast linear system solve
        // x' = M_p_h2L^{-1} (h^2 J d - b)
        x = ldlt.solve(h2J * d - b);
    }
    auto endTime_0 = omp_get_wtime();
    std::cout << "solve time: " << (endTime_0 - stratTime_0) * 1000 << "ms" << std::endl;
    // std::cout << "solve end" << std::endl;
    // Return the solution x
    return x;
}
void RectClothSimulator::applyConstraints() {
    int left_up_index = cloth->idxFromCoord(0, 0);
    int right_up_index = cloth->idxFromCoord(cloth->nw - 1, 0);
    q_n.block<3, 1>(3 * left_up_index, 0) = Eigen::Vector3f(cloth->getInitialPosition(0, 0).x, cloth->getInitialPosition(0, 0).y, cloth->getInitialPosition(0, 0).z);
    q_n.block<3, 1>(3 * right_up_index, 0) = Eigen::Vector3f(cloth->getInitialPosition(cloth->nw - 1, 0).x, cloth->getInitialPosition(cloth->nw - 1, 0).y, cloth->getInitialPosition(cloth->nw - 1, 0).z);
    q_n_1.block<3, 1>(3 * left_up_index, 0) = Eigen::Vector3f(cloth->getInitialPosition(0, 0).x, cloth->getInitialPosition(0, 0).y, cloth->getInitialPosition(0, 0).z);
    q_n_1.block<3, 1>(3 * right_up_index, 0) = Eigen::Vector3f(cloth->getInitialPosition(cloth->nw - 1, 0).x, cloth->getInitialPosition(cloth->nw - 1, 0).y, cloth->getInitialPosition(cloth->nw - 1, 0).z);
}
