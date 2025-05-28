#ifndef _LOW_PASS_FILTER_HPP_
#define _LOW_PASS_FILTER_HPP_

/**
 * First-order low pass fiter.
 */
class LowPassFilter {
   public:
    LowPassFilter(double tau);

    /**
     * Updates the output of the filter
     * @param value The current value of the function.
     * @param interval The time elapsed since the last call to update. Must be strictly positive.
     */
    void update(double input, double interval);

    double value() const;
    operator double() const;

    void reset();

   private:
    double m_tau;     // time constant, in s
    double m_output;  // last output that has been computed
};

#endif
