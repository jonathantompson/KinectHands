#include "math/optimization_test_functions.h"

namespace math {

  void initOptimizationTestFunctions() {
    for (uint32_t i = 0; i < NUM_COEFFS_ROSENBROCK; i++) {
      c_0_rosenbrock(i) = 0;
      c_answer_rosenbrock(i) = 1;
    }

    for (uint32_t i = 0; i < NUM_COEFFS_RASTRIGIN; i++) {
      c_0_rastrigin(i) = 0.1f;
      c_answer_rastrigin(i) = 0;
    }

    for (uint32_t i = 0; i < NUM_COEFFS_ROTELLIPS; i++) {
      c_0_rotellips(i) = 0.1f;
      c_answer_rotellips(i) = 0;
    }

    x_vals_rand << 8.147236863931790f, 9.057919370756192f, 1.269868162935061f, 9.133758561390193f, 6.323592462254095f, 0.975404049994095f, 2.784982188670484f, 5.468815192049839f, 9.575068354342976f, 9.648885351992766f, 1.576130816775483f, 9.705927817606156f, 9.571669482429456f, 4.853756487228412f, 8.002804688888002f, 1.418863386272153f, 4.217612826262750f, 9.157355251890671f, 7.922073295595544f, 9.594924263929030f, 6.557406991565868f, 0.357116785741896f, 8.491293058687772f, 9.339932477575506f, 6.787351548577735f, 7.577401305783335f, 7.431324681249162f, 3.922270195341682f, 6.554778901775567f, 1.711866878115618f, 7.060460880196088f, 0.318328463774207f, 2.769229849608900f, 0.461713906311539f, 0.971317812358475f, 8.234578283272926f, 6.948286229758170f, 3.170994800608606f, 9.502220488383550f, 0.344460805029088f, 4.387443596563982f, 3.815584570930084f, 7.655167881490024f, 7.951999011370631f, 1.868726045543786f, 4.897643957882311f, 4.455862007108995f, 6.463130101112647f, 7.093648308580725f, 7.546866819823609f, 2.760250769985784f, 6.797026768536748f, 6.550980039738406f, 1.626117351946306f, 1.189976815583766f, 4.983640519821430f, 9.597439585160810f, 3.403857266661332f, 5.852677509797774f, 2.238119394911370f, 7.512670593056528f, 2.550951154592691f, 5.059570516651424f, 6.990767226566859f, 8.909032525357985f, 9.592914252054444f, 5.472155299638031f, 1.386244428286791f, 1.492940055590575f, 2.575082541237364f, 8.407172559836626f, 2.542821789715311f, 8.142848260688163f, 2.435249687249893f, 9.292636231872278f, 3.499837659848088f, 1.965952504312082f, 2.510838579760311f, 6.160446761466392f, 4.732888489027292f, 3.516595070629967f, 8.308286278962909f, 5.852640911527242f, 5.497236082911395f, 9.171936638298101f, 2.858390188203735f, 7.572002291107213f, 7.537290942784953f, 3.804458469753567f, 5.678216407252211f, 0.758542895630636f, 0.539501186666072f, 5.307975530089727f, 7.791672301020112f, 9.340106842291830f, 1.299062084737301f, 5.688236608721927f, 4.693906410582058f, 0.119020695012414f, 3.371226443988815f;

    y_vals_rand << -0.027364268160849f,  0.001177123550172f,  0.291174937221837f, -0.004710931293419f, -0.002026559442598f,  0.573265871074401f, -0.168561064783553f, -0.059923834005498f,  0.009860818239772f,  0.020599183044889f, -0.006792580824470f, -0.017222336202960f, -0.005978758533708f, -0.011091638799295f, -0.016085464563436f,  0.156798616943191f,  0.102689812337635f,  0.009133046396285f, -0.022194379735536f,  0.000776782089875f,  0.007564846548581f,  0.577003547312478f, -0.005450509232232f,  0.012206898437902f,  0.017829218330598f,  0.007200537210855f,  0.015485480483131f,  0.151676599059093f,  0.016720925365781f, -0.111278368880478f,  0.008776134724199f,  0.503493637858937f, -0.177963192931801f,  0.617465006973182f,  0.578322759484372f, -0.008415813400939f,  0.030430600859118f, -0.001297903324332f,  0.012608108810081f,  0.538628946738155f,  0.064478527489239f,  0.144967900837960f,  0.005807039847783f, -0.021156331376431f, -0.223324050823138f, -0.039596611739515f,  0.043096535042798f,  0.002345197598618f,  0.023444239086494f, -0.006786318343172f, -0.164142543496834f,  0.033814090369076f,  0.019090031968730f, -0.049317346837546f,  0.372660814600173f, -0.032540166671876f,  0.001457170322907f,  0.084167347528474f, -0.027136610407053f, -0.319770325007174f,  0.008851711292662f, -0.261314534227907f, -0.059463707365263f,  0.018769161763948f,  0.015284176077941f,  0.019280189686598f, -0.061663601044338f,  0.167823186304599f,  0.064860893016818f, -0.251690917543374f, -0.005445148083284f, -0.274446462203868f, -0.032611813083434f, -0.306731901065863f,  0.000828235896325f,  0.118047410058613f, -0.261376627362264f, -0.272738147430737f, -0.009329593473966f, -0.008606722733831f,  0.126093095573388f, -0.025998139419832f, -0.036103472710658f, -0.072503322853813f, -0.008286830528562f, -0.122989185580795f,  0.022517465671294f,  0.002486015237947f,  0.157383238739885f, -0.048110043304297f,  0.682694273218021f,  0.671124858187388f, -0.067536054903381f, -0.000504582119562f, -0.001349005742435f,  0.270593495266512f, -0.045754808313993f,  0.018805053368607f,  0.226845701017717f,  0.080058449404167f;

    c_0_exponential_fit << 1, 1, 1, 1;
    c_answer_exponential_fit << 1.002174785740915f, 0.500160688497305f, 
    2.000464556226078f, 0.003137504833740f;

    c_answer_hw7_4a << 0, 0, 0;
    c_0_hw7_4a << -0.5f, 0.5f, 0.5f;
    
    c_answer_hw7_4b << -1, -1, -0.1f , -0.01f;
    c_0_hw7_4b << -1.02f, -1.02f, -0.122f, -0.102f;
  }

  Eigen::MatrixXf c_0_rosenbrock(NUM_COEFFS_ROSENBROCK, 1);
  Eigen::MatrixXf c_answer_rosenbrock(1, NUM_COEFFS_ROSENBROCK);

  // From: http://www.it.lut.fi/ip/evo/functions/node5.html
  // Unique min at [1, 1, ...., 1]
  float extendedRosenbrock(const Eigen::MatrixXf& coeff) {
    uint32_t n = static_cast<uint32_t>(coeff.size());
    static const float c = 100;
    if (n % 2 != 0) {
      throw std::runtime_error("ERROR: n must be even!");
    }

    float ret_val = 0;
    for (uint32_t i = 1; i <= n-1 ; i++) {
      ret_val += c * (coeff(i-1) * coeff(i-1) - coeff(i)) * 
        (coeff(i-1) * coeff(i-1) - coeff(i)) + 
        (1-coeff(i-1)) * (1-coeff(i-1));
    }
    return ret_val;
  }

  Eigen::MatrixXf c_0_rastrigin(NUM_COEFFS_RASTRIGIN, 1);
  Eigen::MatrixXf c_answer_rastrigin(1, NUM_COEFFS_RASTRIGIN);

  // From: http://www.it.lut.fi/ip/evo/functions/node6.html
  // Unique min at [0, 0, ...., 0]
  float generalizedRastrigin(const Eigen::MatrixXf& coeff) {
    uint32_t n = static_cast<uint32_t>(coeff.size());

    float ret_val = 10.0f * static_cast<float>(n);
    for (uint32_t i = 0; i < n ; i++) {
      ret_val += (coeff(i)*coeff(i) - 10 * cos(2.0f * static_cast<float>(M_PI) * coeff(i)));
    }
    return ret_val;
  }

  Eigen::MatrixXf c_0_rotellips(NUM_COEFFS_ROTELLIPS, 1);
  Eigen::MatrixXf c_answer_rotellips(1, NUM_COEFFS_ROTELLIPS);

  // From: http://www.it.lut.fi/ip/evo/functions/node4.html
  // Unique min at [0, 0, ...., 0]
  float rotatedEllipsoidal(const Eigen::MatrixXf& coeff) {
    uint32_t n = static_cast<uint32_t>(coeff.size());

    float ret_val = 0;
    float running_sum = 0;
    for (uint32_t i = 0; i < n ; i++) {
      running_sum += coeff(i);  // This is the running sum from j=0 to i of x_i
      ret_val += running_sum * running_sum;
    }
    return ret_val;
  }

  Eigen::MatrixXf c_0_exponential_fit(NUM_COEFFS_EXPONTIAL_FIT, 1);
  Eigen::MatrixXf c_answer_exponential_fit(1, NUM_COEFFS_EXPONTIAL_FIT);

  void func(Eigen::MatrixXf& f, const Eigen::MatrixXf& coeff, 
    const Eigen::MatrixXf& x) {
      // From matlab:
      // func = 
      // @(x,c)c(0)*exp(-c(1)*x).*sin(c(2)*x+c(3))
      for (uint32_t i = 0; i < NUM_PTS_EXPONTIAL_FIT; i++) {
        f(i) = coeff(0) * exp(-coeff(1) * x(i)) * sin(coeff(2) * x(i) + coeff(3));
      }
  }

  Eigen::MatrixXf delta_y(NUM_PTS_EXPONTIAL_FIT, 1);
  float residueFunc(const Eigen::MatrixXf& y, const Eigen::MatrixXf& f_x, 
    const Eigen::MatrixXf& coeff) {
      delta_y = f_x - y;
      Eigen::MatrixXf res = delta_y.transpose() * delta_y;  // 1 x 1 matrix
      return res(0);
  }

  Eigen::MatrixXf f(NUM_PTS_EXPONTIAL_FIT, 1);
  Eigen::MatrixXf y_vals_rand(NUM_PTS_EXPONTIAL_FIT, 1);
  Eigen::MatrixXf x_vals_rand(NUM_PTS_EXPONTIAL_FIT, 1);

  float exponentialFit(const Eigen::MatrixXf& coeff) {
    // Evaluate the function
    func(f, coeff, x_vals_rand);
    // Calculate the residual
    return residueFunc(y_vals_rand, f, coeff);
  }

  Eigen::MatrixXf c_0_hw7_4a(NUM_COEFFS_HW7_4A, 1);
  Eigen::MatrixXf c_answer_hw7_4a(1, NUM_COEFFS_HW7_4A);

  float hw7_4a(const Eigen::MatrixXf& coeff) {
    float cos_x_0 = cosf(coeff(0));
    return coeff(0) * coeff(0) * expf(coeff(2)) + coeff(1)*coeff(1) + 
      cos_x_0*cos_x_0*coeff(2)*coeff(2);
  }

  void hw7_4a_jacob(Eigen::MatrixXf& jacob, const Eigen::MatrixXf& coeff) {
    float cos_x_0 = cosf(coeff(0));
    jacob(0) = 2.0f*coeff(0)*expf(coeff(2)) - 2*coeff(2)*coeff(2)*cos_x_0*sinf(coeff(0));
    jacob(1) = 2.0f*coeff(1);
    jacob(2) = 2.0f*coeff(2)*cos_x_0*cos_x_0 + coeff(0)*coeff(0)*exp(coeff(2));
  }
  
  Eigen::MatrixXf c_0_hw7_4b(NUM_COEFFS_HW7_4B, 1);
  Eigen::MatrixXf c_answer_hw7_4b(1, NUM_COEFFS_HW7_4B);
  
  float hw7_4b(const Eigen::MatrixXf& coeff) {
    return coeff(0) + coeff(1) + coeff(2) + coeff(3) + 0.5f*(coeff(0)*coeff(0) +
      coeff(1)*coeff(1) + 10.0f*coeff(2)*coeff(2) + 100.0f*coeff(3)*coeff(3));
  }
  
  void hw7_4b_jacob(Eigen::MatrixXf& jacob, const Eigen::MatrixXf& coeff) {
    jacob(0) = coeff(0) + 1.0f;
    jacob(1) = coeff(1) + 1.0f;
    jacob(2) = 10.0f*coeff(2) + 1.0f;
    jacob(3) = 100.0f*coeff(3) + 1.0f;
  }
}  // namespace math