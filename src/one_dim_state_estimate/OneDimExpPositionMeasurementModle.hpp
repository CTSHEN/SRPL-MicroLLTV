#ifndef ONE_DIM_EXP_POSITIONMEASUREMENTMODEL_HPP_
#define ONE_DIM_EXP_POSITIONMEASUREMENTMODEL_HPP_

#include <LinearizedMeasurementModel.hpp>

namespace OneDimKalman
{
    /**
     * @brief Measurement vector (actually a scalar) measuring height
     * 
     * @param T Numeric scalar type
     */
    template<typename T>
    class PositionMeasurement : public Kalman::Vector<T,1>
    {
        public:
            KALMAN_VECTOR(PositionMeasurement, T, 1)
            //! Height Measurement
            static constexpr size_t HM = 0;

            T hm()  const{ return (*this)[ HM ];}

            T& hm() { return (*this)[ HM ];}
    };

    /**
     * @brief Measurement Model
     * 
     * The lidar measurement will directly measure the height, which is one of our state.
     * 
     * @param T Numeric scalar type
     * @param CovarianceBase Class template to determine the covariance representation
     *                       (as covariance matrix (StandardBase) or as lower-triangular
     *                       coveriace square root (SquareRootBase)) 
     * 
     */
    template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
    class PositionMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, PositionMeasurement<T>, CovarianceBase>
    {
        public:
            //! State type shortcut definition
            typedef OneDimKalman::State<T> S;
            //! Measurement type shortcut definition
            typedef OneDimKalman::PositionMeasurement<T> M;

            // Constructor
            PositionMeasurementModel()
            {
                //Setup noise jacobian. As this one is static, we can define it once
                // and do not need to update it dynamically.
                this->V.setIdentity();

            }

        protected:
            /**
             * @brief Update jacobian matrix for the measurement model using surrent state
             * 
             * Actually this OneDimExp has a linear measurement model, which is:
             *                   _   _
             *        _       _ |  h  |
             *  hm = |_ 1 0 0 _||  v  |
             *                  |_ fm_|
             *         
             * @param x The current system state around which to linearized          
             */
            void updateJacobians( const S& x)
            {
                this -> H.setZero();

                this ->H( M::HM, S::H) = 1;
                this ->H( M::HM, S::V) = 0;
                this ->H( M::HM, S::FM) =0;
            }

    };
} // namespace OneDimKalman

#endif