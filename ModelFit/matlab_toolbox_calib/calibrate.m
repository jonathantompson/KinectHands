addpath([pwd(), '\toolbox_calib']);
calib_gui 

format long e
film_size_horiz = 640;
focal_length_horiz = 573.89093;
focal_length_uncertainty_horiz = 2.53706;
film_size_vert = 480;
focal_length_vert = 571.62591;
focal_length_uncertainty_vert = 2.50702;
FOV_horiz = 2 * atan (film_size_horiz / (2*focal_length_horiz))
FOV_vert = 2 * atan (film_size_vert / (2*focal_length_vert))

FOV_horiz_plus = 2 * atan (film_size_horiz / (2*(focal_length_horiz + focal_length_uncertainty_horiz)))
FOV_vert_plus = 2 * atan (film_size_vert / (2*(focal_length_vert + focal_length_uncertainty_vert)))

FOV_horiz_minus = 2 * atan (film_size_horiz / (2*(focal_length_horiz - focal_length_uncertainty_horiz)))
FOV_vert_minus = 2 * atan (film_size_vert / (2*(focal_length_vert - focal_length_uncertainty_vert)))
