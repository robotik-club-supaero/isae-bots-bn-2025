#include "math/RollingMax.hpp"
#include <limits>

RollingMax::RollingMax(const std::function<double_t(double_t)> &function, double_t domainStart, double_t domainEnd, size_type numOfSteps)
    : m_samples(), m_subdomains() {
    std::vector<Evaluation> samples;
    samples.reserve(numOfSteps + 1);
    double_t step = (domainEnd - domainStart) / numOfSteps;

    MonotonicityKind currentMonotonicity = CONSTANT;
    double_t oldValue;
    for (size_type i = 0; i <= numOfSteps; i++) {
        double_t value = function(domainStart + i * step);
        if (i > 0) {
            MonotonicityKind monotonicity = (value == oldValue) ? CONSTANT : ((value > oldValue) ? INCREASING : DECREASING);
            if (currentMonotonicity != monotonicity && monotonicity != CONSTANT) {
                if (currentMonotonicity != CONSTANT) {
                    m_subdomains.push_back({currentMonotonicity, i - 1});
                }
                currentMonotonicity = monotonicity;
            }
        }
        samples.push_back({value, m_subdomains.size()});
        oldValue = value;
    }
    m_subdomains.push_back({currentMonotonicity, samples.size() - 1});
    m_samples = Samples(std::move(samples), domainStart, domainEnd);
}

double_t RollingMax::getMaximum(double_t start, double_t end) const {
    size_type startIndex = m_samples.indexOf(start);
    size_type endIndex = m_samples.indexOf(end);

    Evaluation startEval = m_samples[startIndex];
    Evaluation endEval = m_samples[endIndex];

    double_t maxOnInterval = -std::numeric_limits<double_t>::max();
    for (size_type i = startEval.subdomainIndex; i <= endEval.subdomainIndex; i++) {
        Subdomain domain = m_subdomains[i];

        double_t maxOnDomain;
        if (domain.kind == INCREASING) {
            maxOnDomain = m_samples[std::min(endIndex, domain.end)].value;
        } else {
            maxOnDomain = m_samples[startIndex].value;
        }
        startIndex = domain.end + 1;

        maxOnInterval = std::max(maxOnInterval, maxOnDomain);
    }
    return maxOnInterval;
}

double_t RollingMax::operator()(double_t arg) const {
    return m_samples(arg).value;
}

double_t RollingMax::domainStart() const {
    return m_samples.domainStart();
}
double_t RollingMax::domainEnd() const {
    return m_samples.domainEnd();
}
RollingMax::size_type RollingMax::numOfSamples() const {
    return m_samples.numOfSamples();
}