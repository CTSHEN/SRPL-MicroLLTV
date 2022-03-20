#ifndef ONE_DIM_EXP_SYSTEMMODEL_HPP_
#define ONE_DIM_EXP_SYSTEMMODEL_HPP_

#include <LinearizedSystemModel.hpp>

#define FUEL_FLOWRATE 0.018 // 18g/s

namespace OneDimKalman
{
    /**
     * @brief System state vector-type for one-dim exp
     * 
     * This is a system state vector for a one-dimensional
     * height control system that is characterized by HEIGHT, 
     * VELOCITY and FUEL_MASS.
     * 
     * @param T Numeric scalar type
     */
    template<typename T>
    class State : public Kalman::Vector<T, 3>
    {
        public:
            KALMAN_VECTOR(State, T, 3)
            
            //! HEIGHT
            static constexpr size_t H = 0;
            //! VELOCITY
            static constexpr size_t V = 1;
            //! FUEL_MASS
            static constexpr size_t FM = 2;

            T h() const { return (*this)[ H ]; }
            T v() const { return (*this)[ V ]; }
            T fm() const{ return (*this)[ FM ];}

            T& h() {return (*this)[ H ]; }
            T& v() {return (*this)[ V ]; }
            T& fm(){return (*this)[ FM ];}
    };

    /**
     * @brief Observer input vetcor-type for one-dim exp
     * 
     * This is the input of the kinematic model for one-dim
     * height experiment, that is the acceleration along the 
     * z-axis.
     * The on-off signal of two thrusters affect the fuel mass.
     * 
     * @param T Numeric scalar type
     */
    template<typename T>
    class Input : public Kalman::Vector<T, 3>
    {
        public:
        KALMAN_VECTOR( Input, T, 3)

        //! Z acceleration
        static constexpr size_t AZ = 0;
        //! dowmward thruster
        static constexpr size_t TD = 1;
        //! upward thruster
        static constexpr size_t TU = 2;


        T az() const {return (*this)[ AZ ]; }
        T td() const {return (*this)[ TD ]; }
        T tu() const {return (*this)[ TU ]; }

        T& az() {return (*this)[ AZ ]; }
        T& td() {return (*this)[ TD ]; }
        T& tu() {return (*this)[ TU ]; }
    };

    /**
     * @brief 1-D translation kinematic model
     * 
     * x_k+1 = x_k + v_k*dt + 0.5*a_k*dt^2
     * v_k+1 = v_k + a_k*dt
     * fm_k+1 = fm_k - FUEL_FLOWRATE*dt*(td + tu)
     * 
     * @param T Numeric scalar type
     * @param CovarianceBase Class template to determine the covariance representation
     *                       (as covariance matrix (StandardBase) or as lower-triangular
     *                       coveriace square root (SquareRootBase))
     * 
     */
    template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
    class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Input<T>, CovarianceBase>
    {
        public:
        //! State type shortcut definition
        typedef OneDimKalman::State<T> S;
        //! Input typoe shortcut definition
        typedef OneDimKalman::Input<T> I;
        //! Define and initialize the predict time interval
        // Although the system is runnung at a constant sampling time
        // i.e. the sampling time of the accelerometer, but the acc input
        // signal and measurement update signal is asynchronized, a small 
        // state propagation is needed before the measurement update.
        // Therefore the "dt" need to be calculated every time before predict or update.
        double dt = 0.1;
        double timeStamp_now;

        /**
         * @brief Update system timeStamp_now
         * 
         * This system timeStamp_now is used to compute the ekf predict
         * time interval.
         * The predict time interval dt = (input time stamp) - timeStamp_now
         * is computed before a prediction procedure every time a input signal
         * arrives.
         * 
         * @param [in] t The current timestamp .
         */
        void setupSystemTime( double t)
        {
            timeStamp_now = t;
        }

        /**
         * @brief Definition of translation kinematics function
         * 
         * This function defines the station transition used in
         * propagation stage.
         * 
         * @param [in] x The system state in current time-step
         * @param [in] u The input vector of the observer (which is
         *                  the z-axis acceleration.)
         * @param [in] dt The sampling time interval.
         * @returns The predicted system state in the next time step. 
         */
        S f(const S& x, const I& u) const
        {
            //! predicted state vector
            S x_;

            x_.h() = x.h() + x.v()*dt + 0.5*u.az()*(dt^2);
            x_.v() = x.v() + u.az()*dt;
            x_.fm() = x.fm() - FUEL_FLOWRATE*dt*(u.td() + u.tu());

            // Return predicted state vector
            return x_;
        }

        protected:
        /**
         * @brief Update Jacobian matrices for the system state transition function
         * Actually it's already a linear model:
         *  _       _    _         _  _     _    _               _  _       _
         * | h_k+1   |  | 1  dt  0  ||  h_k  |  |  0.5*dt^2  0  0 ||   a_k   |
         * | v_k+1   | =| 0   1  0  ||  v_k  | +|     dt     0  0 ||  T_d,k  |
         * |_fm_k+1 _|  |_0   0  1 _||_ fm_k_|  |_    dt     0  0_||_ T_u,k _|
         * 
         * @param x the current system state
         * @param u Ther current inputs
         */
        void updateJacobians( const S& x, const C& u)
        {
            // F = df/dx  (actually it's already a linear model)
            this->F.setZero();

            this->F( S::H, S::H) = 1;
            this->F( S::H, S::V) = dt;

            this->F( S::V, S::V) = 1;

            this->F(S::FM, S::FM) = 1;

            // W = df/dw (Jacobian of state transition w.r.t. the noise)
            this -> W.setIdentity();
        }
    };
} // namespace OneDimKalman

#endif