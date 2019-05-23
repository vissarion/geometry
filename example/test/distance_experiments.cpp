#include <iostream>
#include <sys/time.h>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>
#include <boost/lexical_cast.hpp>

#include <boost/geometry.hpp>

#include <boost/geometry/formulas/elliptic_arc_length.hpp>
#include <boost/geometry/formulas/flat_earth_approximation.hpp>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/min.hpp>

#include <boost/geometry/srs/projection.hpp>
#include <boost/geometry/srs/transformation.hpp>
#include <boost/geometry/strategies/transform/srs_transformer.hpp>

#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/Constants.hpp>

//#define PRINTDATA
#define STATS

/** Data from here: https://zenodo.org/record/32156#.WOyaMrUmv7C
 *
  Each line of the test set gives 10 space delimited numbers

  0 latitude at point 1, φ1 (degrees, exact)
  1 longitude at point 1, λ1 (degrees, always 0)
  2 azimuth at point 1, α1 (clockwise from north in degrees, exact)
  3 latitude at point 2, φ2 (degrees, accurate to 10−18 deg)
  4 longitude at point 2, λ2 (degrees, accurate to 10−18 deg)
  5 azimuth at point 2, α2 (degrees, accurate to 10−18 deg)
  6 geodesic distance from point 1 to point 2, s12 (meters, exact)
  7 arc distance on the auxiliary sphere, σ12 (degrees, accurate to 10−18 deg)
  8 reduced length of the geodesic, m12 (meters, accurate to 0.1 pm)
  9 the area between the geodesic and the equator, S12 (m2, accurate to 1 mm2)
 **/

namespace bg = boost::geometry;
namespace ba = boost::accumulators;

template<typename Out>
void split(const std::string &s, Out result)
{
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, ' '))
    {
        try
        {
            *(result++) = boost::lexical_cast<double>(item);
        }
        catch (boost::bad_lexical_cast const&)
        {
            *(result++) = 0;
        }
    }
}

template<size_t SIZE, class T> inline size_t array_size(T (&arr)[SIZE]) {
    return SIZE;
}

int main()
{
    //for(int k=13; k>=0; k--)
    //for(int k=0; k<14; k++)
    //{

    typedef double CT;
    //typedef boost::multiprecision::cpp_dec_float_50 MCT;

    std::string line;
    std::ifstream myfile ("/home/vissarion/Documents/GIS_algorithms_implementations/data/GeodTest.dat");

    std::cout.precision(10);

    if (myfile.is_open())
    {
        int count=0, visited=0, num_of_exp=1;

        size_t const num_of_methods = 18;
        size_t const num_of_values = 7;
        size_t const num_of_geo_categories = 15; // w.r.t. distance

        //vinc, thomas, and,
        //karn1, karn4, karn8
        //ell0, ell1, ell2, ell3, ell4,
        //meridian ell0, ell1, ell2, ell3,
        //flat, sph
        //geographiclib
        bool enable_method[num_of_methods]
                = {true, true, true,
                   true, true, true,
                   false, false, false, false, false,
                   false, false, false, false,
                   true, true,
                   true};
        //std::fill_n(enable_method, num_of_methods, false);
        //enable_method[k] = true;
        double times[num_of_methods];
        std::fill_n(times, num_of_methods, 0);

        const bool EnableDistance = false,
             EnableAzimuth = true,
             EnableReverseAzimuth = true,
             EnableReducedLength = false,
             EnableGeodesicScale = false,
             EnableArea = false;

        // Define an accumulator set for calculating the mean, min, max
        typedef typename ba::accumulator_set
                <
                CT,
                ba::stats
                <
                ba::tag::mean,
                ba::tag::max,
                ba::tag::min
                >
                > accumulator_set;
        accumulator_set acc[num_of_methods][num_of_values][num_of_geo_categories];

        while ( std::getline(myfile, line) )
        {
            //read data from file
            std::vector<CT> data;
            split(line, std::back_inserter(data));
            //if(count < 100000)
            //-->//if (data[4]<90 && count < 200000)// && std::abs(data[3]-data[0])>0.1)// &&
            if ((count<100000) || (count >150000 && count < 200000) )// && std::abs(data[3]-data[0])>0.1)// &&
            //if ((count<100))// || (count >150000 && count < 200000) )// && std::abs(data[3]-data[0])>0.1)// &&
                          //if (data[6]<500 && data[0]<80)// &&
            {
#ifdef PRINTDATA
                std::cout.precision(10);
                std::cout << count << ": ";
                std::cout << data[1] << "," << data[0] << " " <<
                                        data[4] << "," << data[3] << " --> ";
                std::cout << data[6] << " ";
                std::cout << std::abs(data[3]-data[0]);
                std::cout << std::endl;
#endif
                //compute
                visited++;

                CT const d2r = bg::math::d2r<CT>();
                CT const r2d = bg::math::r2d<CT>();

                // WGS84
                //bg::srs::spheroid<double> spheroid(6378137.0, 6356752.3142451793);
                bg::srs::spheroid<CT> spheroid;

                bg::formula::result_inverse<CT> results[num_of_methods];
                CT areas[num_of_methods];

                CT lon1 = data[1] * d2r;
                CT lat1 = data[0] * d2r;
                CT lon2 = data[4] * d2r;
                CT lat2 = data[3] * d2r;

                unsigned int index=0;

                if (enable_method[index])
                {
                    auto t0 = std::chrono::high_resolution_clock::now();
                    for (int i=0; i<num_of_exp; i++)
                    {
                        typedef bg::formula::vincenty_inverse<CT, EnableDistance, EnableAzimuth, EnableReverseAzimuth, EnableReducedLength, EnableGeodesicScale> inv;
                        results[index] = inv::apply(lon1, lat1, lon2, lat2, spheroid);
                    }
                    auto t1 = std::chrono::high_resolution_clock::now();
                    times[index] += double(std::chrono::duration_cast<std::chrono::nanoseconds>
                                       (t1 - t0).count())/num_of_exp;
                }
                index++;
                if (enable_method[index])
                {
                    auto t0 = std::chrono::high_resolution_clock::now();
                    for (int i=0; i<num_of_exp; i++)
                    {
                        typedef bg::formula::thomas_inverse<CT, EnableDistance, EnableAzimuth, EnableReverseAzimuth, EnableReducedLength, EnableGeodesicScale> inv;
                        results[index] = inv::apply(lon1, lat1, lon2, lat2, spheroid);
                    }
                    auto t1 = std::chrono::high_resolution_clock::now();
                    times[index] += double(std::chrono::duration_cast<std::chrono::nanoseconds>
                                       (t1 - t0).count())/num_of_exp;
                }
                index++;
                if (enable_method[index])
                {
                    auto t0 = std::chrono::high_resolution_clock::now();
                    for (int i=0; i<num_of_exp; i++)
                    {
                        typedef bg::formula::andoyer_inverse<CT, EnableDistance, EnableAzimuth, EnableReverseAzimuth, EnableReducedLength, EnableGeodesicScale> inv;
                        results[index] = inv::apply(lon1, lat1, lon2, lat2, spheroid);
                    }
                    auto t1 = std::chrono::high_resolution_clock::now();
                    times[index] += double(std::chrono::duration_cast<std::chrono::nanoseconds>
                                       (t1 - t0).count())/num_of_exp;
                }
                //karn
                index++;
                if (enable_method[index])
                {
                    auto t0 = std::chrono::high_resolution_clock::now();
                    for (int i=0; i<num_of_exp; i++)
                    {
                        typedef bg::formula::karney_inverse<CT, EnableDistance, EnableAzimuth, EnableReverseAzimuth, EnableReducedLength, EnableGeodesicScale, 1> inv;
                        results[index] = inv::apply(lon1, lat1, lon2, lat2, spheroid);
                    }
                    auto t1 = std::chrono::high_resolution_clock::now();
                    times[index] += double(std::chrono::duration_cast<std::chrono::nanoseconds>
                                       (t1 - t0).count())/num_of_exp;
                }
                index++;
                if (enable_method[index])
                {
                    auto t0 = std::chrono::high_resolution_clock::now();
                    for (int i=0; i<num_of_exp; i++)
                    {
                        typedef bg::formula::karney_inverse<CT, EnableDistance, EnableAzimuth, EnableReverseAzimuth, EnableReducedLength, EnableGeodesicScale, 4> inv;
                        results[index] = inv::apply(lon1, lat1, lon2, lat2, spheroid);
                    }
                    auto t1 = std::chrono::high_resolution_clock::now();
                    times[index] += double(std::chrono::duration_cast<std::chrono::nanoseconds>
                                       (t1 - t0).count())/num_of_exp;
                }
                index++;
                if (enable_method[index])
                {
                    auto t0 = std::chrono::high_resolution_clock::now();
                    for (int i=0; i<num_of_exp; i++)
                    {
                        typedef bg::formula::karney_inverse<CT, EnableDistance, EnableAzimuth, EnableReverseAzimuth, EnableReducedLength, EnableGeodesicScale, 8> inv;
                        results[index] = inv::apply(lon1, lat1, lon2, lat2, spheroid);
                    }
                    auto t1 = std::chrono::high_resolution_clock::now();
                    times[index] += double(std::chrono::duration_cast<std::chrono::nanoseconds>
                                       (t1 - t0).count())/num_of_exp;
                }
                /// ell
                index++;
                if (enable_method[index])
                {
                    auto t0 = std::chrono::high_resolution_clock::now();
                    for (int i=0; i<num_of_exp; i++)
                    {
                        //typedef bg::formula::elliptic_arc_length<CT, EnableDistance, EnableAzimuth, EnableReverseAzimuth, EnableReducedLength, EnableGeodesicScale, 0> inv;
                        //results[3] = inv::apply(lon1, lat1, lon2, lat2, spheroid);
                    }
                    auto t1 = std::chrono::high_resolution_clock::now();
                    times[index] += double(std::chrono::duration_cast<std::chrono::nanoseconds>
                                       (t1 - t0).count())/num_of_exp;
                }
                index++;
                if (enable_method[index])
                {
                    auto t0 = std::chrono::high_resolution_clock::now();
                    for (int i=0; i<num_of_exp; i++)
                    {
                        //typedef bg::formula::elliptic_arc_length<CT, EnableDistance, EnableAzimuth, EnableReverseAzimuth, EnableReducedLength, EnableGeodesicScale, 1> inv;
                        //results[4] = inv::apply(lon1, lat1, lon2, lat2, spheroid);
                    }
                    auto t1 = std::chrono::high_resolution_clock::now();
                    times[index] += double(std::chrono::duration_cast<std::chrono::nanoseconds>
                                       (t1 - t0).count())/num_of_exp;
                }
                index++;
                if (enable_method[index])
                {
                    auto t0 = std::chrono::high_resolution_clock::now();
                    for (int i=0; i<num_of_exp; i++)
                    {
                        //typedef bg::formula::elliptic_arc_length<CT, EnableDistance, EnableAzimuth, EnableReverseAzimuth, EnableReducedLength, EnableGeodesicScale, 2> inv;
                        //results[5] = inv::apply(lon1, lat1, lon2, lat2, spheroid);
                    }
                    auto t1 = std::chrono::high_resolution_clock::now();
                    times[index] += double(std::chrono::duration_cast<std::chrono::nanoseconds>
                                       (t1 - t0).count())/num_of_exp;
                }
                index++;
                if (enable_method[index])
                {
                    auto t0 = std::chrono::high_resolution_clock::now();
                    for (int i=0; i<num_of_exp; i++)
                    {
                        //typedef bg::formula::elliptic_arc_length<CT, EnableDistance, EnableAzimuth, EnableReverseAzimuth, EnableReducedLength, EnableGeodesicScale, 3> inv;
                        //results[6] = inv::apply(lon1, lat1, lon2, lat2, spheroid);
                    }
                    auto t1 = std::chrono::high_resolution_clock::now();
                    times[index] += double(std::chrono::duration_cast<std::chrono::nanoseconds>
                                       (t1 - t0).count())/num_of_exp;
                }
                index++;
                if (enable_method[index])
                {
                    auto t0 = std::chrono::high_resolution_clock::now();
                    for (int i=0; i<num_of_exp; i++)
                    {
                        //typedef bg::formula::elliptic_arc_length<CT, EnableDistance, EnableAzimuth, EnableReverseAzimuth, EnableReducedLength, EnableGeodesicScale, 4> inv;
                        //results[7] = inv::apply(lon1, lat1, lon2, lat2, spheroid);
                    }
                    auto t1 = std::chrono::high_resolution_clock::now();
                    times[index] += double(std::chrono::duration_cast<std::chrono::nanoseconds>
                                       (t1 - t0).count())/num_of_exp;
                }
                /// ell-mer
                index++;
                if (enable_method[index])
                {
                    auto t0 = std::chrono::high_resolution_clock::now();
                    for (int i=0; i<num_of_exp; i++)
                    {
                        //typedef bg::formula::elliptic_arc_length<CT, EnableDistance, EnableAzimuth, EnableReverseAzimuth, EnableReducedLength, EnableGeodesicScale, 1> inv;
                        //results[8] = inv::apply_meridian_formula(lon1, lat1, lon2, lat2, spheroid);
                    }
                    auto t1 = std::chrono::high_resolution_clock::now();
                    times[index] += double(std::chrono::duration_cast<std::chrono::nanoseconds>
                                       (t1 - t0).count())/num_of_exp;
                }
                index++;
                if (enable_method[index])
                {
                    auto t0 = std::chrono::high_resolution_clock::now();
                    for (int i=0; i<num_of_exp; i++)
                    {
                        //typedef bg::formula::elliptic_arc_length<CT, EnableDistance, EnableAzimuth, EnableReverseAzimuth, EnableReducedLength, EnableGeodesicScale, 2> inv;
                        //results[9] = inv::apply_meridian_formula(lon1, lat1, lon2, lat2, spheroid);
                    }
                    auto t1 = std::chrono::high_resolution_clock::now();
                    times[index] += double(std::chrono::duration_cast<std::chrono::nanoseconds>
                                       (t1 - t0).count())/num_of_exp;
                }
                index++;
                if (enable_method[index])
                {
                    auto t0 = std::chrono::high_resolution_clock::now();
                    for (int i=0; i<num_of_exp; i++)
                    {
                        //typedef bg::formula::elliptic_arc_length<CT, EnableDistance, EnableAzimuth, EnableReverseAzimuth, EnableReducedLength, EnableGeodesicScale, 3> inv;
                        //results[10] = inv::apply_meridian_formula(lon1, lat1, lon2, lat2, spheroid);
                    }
                    auto t1 = std::chrono::high_resolution_clock::now();
                    times[index] += double(std::chrono::duration_cast<std::chrono::nanoseconds>
                                       (t1 - t0).count())/num_of_exp;
                }
                index++;
                if (enable_method[index])
                {
                    auto t0 = std::chrono::high_resolution_clock::now();
                    for (int i=0; i<num_of_exp; i++)
                    {
                        //typedef bg::formula::elliptic_arc_length<CT, EnableDistance, EnableAzimuth, EnableReverseAzimuth, EnableReducedLength, EnableGeodesicScale, 4> inv;
                        //results[11] = inv::apply_meridian_formula(lon1, lat1, lon2, lat2, spheroid);
                    }
                    auto t1 = std::chrono::high_resolution_clock::now();
                    times[index] += double(std::chrono::duration_cast<std::chrono::nanoseconds>
                                       (t1 - t0).count())/num_of_exp;
                }
                index++;
                if (enable_method[index])
                {
                    auto t0 = std::chrono::high_resolution_clock::now();
                    for (int i=0; i<num_of_exp; i++)
                    {
                        typedef bg::formula::flat_earth_approximation<CT, EnableDistance, EnableAzimuth, EnableReverseAzimuth, EnableReducedLength, EnableGeodesicScale> inv;
                        results[index] = inv::apply(lon1, lat1, lon2, lat2, spheroid);
                    }
                    auto t1 = std::chrono::high_resolution_clock::now();
                    times[index] += double(std::chrono::duration_cast<std::chrono::nanoseconds>
                                       (t1 - t0).count())/num_of_exp;
                }
                index++;
                if (enable_method[index])
                {
                    auto t0 = std::chrono::high_resolution_clock::now();
                    //CT r = bg::get_radius<0>(bg::srs::sphere<CT>());
                    CT r = bg::formula::mean_radius<double>(bg::srs::spheroid<double>());
                    //CT r = (6356752.3142451793);
                    for (int i=0; i<num_of_exp; i++)
                    {
                        CT const a = bg::math::hav(lat2 - lat1) + cos(lat1) * cos(lat2) * bg::math::hav(lon2 - lon1);
                        CT const c = 2.0 * asin(bg::math::sqrt(a));
                        //typedef bg::model::point<CT, 2,
                        //        bg::cs::spherical_equatorial<bg::degree> > point_sph;
                        //bg::strategy::distance::haversine<CT> haversine(6378137);
                        //results[9].distance = haversine.apply(point_sph(data[1], data[0]),
                        //        point_sph(data[4], data[3]));
                        results[index].distance = r * c;
                        auto result = bg::formula::spherical_azimuth<CT, EnableReverseAzimuth>(lon1, lat1, lon2, lat2);
                        results[index].azimuth = result.azimuth;

                    }
                    auto t1 = std::chrono::high_resolution_clock::now();
                    times[index] += double(std::chrono::duration_cast<std::chrono::nanoseconds>
                                       (t1 - t0).count())/num_of_exp;
                }
                index++;
                if (enable_method[index])
                {
                    auto t0 = std::chrono::high_resolution_clock::now();
                    using namespace GeographicLib;
                    Geodesic geod(Constants::WGS84_a(), Constants::WGS84_f());
                    double s12, azi1, azi2, m12, M12, M21, S12;
                    for (int i=0; i<num_of_exp; i++)
                    {
                        geod.Inverse(lat1*r2d, lon1*r2d, lat2*r2d, lon2*r2d, s12, azi1, azi2, m12, M12, M21, S12);
                        results[index].distance = s12;
                        results[index].azimuth = azi1*d2r;
                        areas[num_of_methods-1] = S12;
                    }
                    auto t1 = std::chrono::high_resolution_clock::now();
                    times[index] += double(std::chrono::duration_cast<std::chrono::nanoseconds>
                                       (t1 - t0).count())/num_of_exp;
                }
                index++;

                /* AREA */
                if(EnableArea)
                {
                    bg::strategy::area::geographic<bg::strategy::vincenty> vi_s;
                    bg::strategy::area::geographic<bg::strategy::thomas> th_s;
                    bg::strategy::area::geographic<bg::strategy::andoyer> an_s;
/*                    bg::strategy::area::geographic
                            <
                            bg::strategy::karney,
                            5,
                            bg::srs::spheroid<CT>,
                            CT
                            > ka_s;
*/
                    typedef bg::model::point<CT, 2, bg::cs::geographic<bg::degree> > point;

                    bg::model::polygon<point> poly{{{data[1], data[0]},
                            {data[4], data[3]},
                            {data[4], 0.0},
                            {data[1], 0.0}}};



                    {
                        auto t0 = std::chrono::high_resolution_clock::now();
                        for (int i=0; i<num_of_exp; i++)
                        {
                            areas[0] = bg::area(poly, vi_s);
                        }
                        auto t1 = std::chrono::high_resolution_clock::now();
                        times[7] += double(std::chrono::duration_cast<std::chrono::nanoseconds>
                                           (t1 - t0).count())/num_of_exp;
                    }{
                        auto t0 = std::chrono::high_resolution_clock::now();
                        for (int i=0; i<num_of_exp; i++)
                        {
                            areas[1] = bg::area(poly, th_s);
                        }
                        auto t1 = std::chrono::high_resolution_clock::now();
                        times[8] += double(std::chrono::duration_cast<std::chrono::nanoseconds>
                                           (t1 - t0).count())/num_of_exp;
                    }{
                        auto t0 = std::chrono::high_resolution_clock::now();
                        for (int i=0; i<num_of_exp; i++)
                        {
                            areas[2] = bg::area(poly, an_s);
                        }
                        auto t1 = std::chrono::high_resolution_clock::now();
                        times[9] += double(std::chrono::duration_cast<std::chrono::nanoseconds>
                                           (t1 - t0).count())/num_of_exp;
                    }{
                        auto t0 = std::chrono::high_resolution_clock::now();
                        for (int i=0; i<num_of_exp; i++)
                        {
       //                     areas[3] = bg::area(poly, ka_s);
                        }
                        auto t1 = std::chrono::high_resolution_clock::now();
                        times[10] += double(std::chrono::duration_cast<std::chrono::nanoseconds>
                                            (t1 - t0).count())/num_of_exp;
                    }
                    {
                        auto t0 = std::chrono::high_resolution_clock::now();
                        for (int i=0; i<num_of_exp; i++)
                        {
                            // Make a projection (~equal area)
                            typedef bg::projections::parameters<CT> parameters_type;
                            //typedef bg::projections::sterea_ellipsoid<CT, parameters_type> projection_type;
                            typedef bg::projections::cea_ellipsoid<CT, parameters_type> projection_type;

                            //char buffer [50];
                            //std::sprintf(buffer, "+lat_0=%f", data[0]);

                            bg::srs::detail::proj4_parameters params("");
                            //            "+lat_0=52.15616055555555 +lon_0=5.38763888888889 +k=0.9999079 +x_0=155000 +y_0=463000 +ellps=bessel +units=m");
                            parameters_type par = bg::projections::detail::pj_init<CT>(params);
                            bg::strategy::transform::srs_forward_transformer<projection_type> transformer(params, par);

                            typedef bg::model::d2::point_xy<CT> cartesian_point_type;
                            typedef bg::model::polygon<cartesian_point_type> cartesian_polygon_type;
                            cartesian_polygon_type cart_poly;

                            bg::transform(poly, cart_poly, transformer);
                            areas[4] = bg::area(cart_poly);
                        }
                        auto t1 = std::chrono::high_resolution_clock::now();
                        times[11] += double(std::chrono::duration_cast<std::chrono::nanoseconds>
                                            (t1 - t0).count())/num_of_exp;
                    }

                }
                /*
                bg::model::polygon<point_sph> poly_sph{{{data[1], data[0]},
                        {data[4], data[3]},
                        {data[4], 0.0},
                        {0.0, 0.0}}};

                bg::strategy::area::spherical
                        <
                        typename bg::point_type<point_sph>::type
                        > area_spherical(6378137);

                areas[3] = bg::area(poly_sph, area_spherical);
                */
#ifdef STATS
                for (size_t i = 0; i < num_of_methods; i++)
                {
                    size_t res_index = 0;
                    if (count > 150000 && count < 200000 && data[6] < 2) res_index = 0;
                    else if (count > 150000 && count < 200000 && data[6] < 10) res_index = 1;
                    else if (count > 150000 && count < 200000 && data[6] < 100) res_index = 2;
                    else if (count > 150000 && count < 200000 && data[6] < 1000) res_index = 3;
                    else if (count > 150000 && count < 200000 && data[6] < 2000) res_index = 4;
                    else if (count < 100000 && data[6] < 1000000) res_index = 5;
                    else if (count < 100000 && data[6] < 2000000) res_index = 6;
                    else if (count < 100000 && data[6] < 5000000) res_index = 7;
                    else if (count < 100000 && data[6] < 7500000) res_index = 8;
                    else if (count < 100000 && data[6] < 10000000) res_index = 9;
                    else if (count < 100000 && data[6] < 12500000) res_index = 10;
                    else if (count < 100000 && data[6] < 15000000) res_index = 11;
                    else if (count < 100000 && data[6] < 17500000) res_index = 12;
                    else if (count < 100000 && data[6] < 19500000) res_index = 13;
                    else if (count < 100000) res_index = 14;

                    if(EnableDistance)
                    {
                        acc[i][0][res_index](bg::math::abs(results[i].distance - data[6]));
                        acc[i][1][res_index](bg::math::abs((results[i].distance - data[6])/data[6]));
                    }
                    if(EnableAzimuth)
                    {
                        acc[i][0][res_index](bg::math::abs(results[i].azimuth * r2d - data[2]));
                        acc[i][1][res_index](bg::math::abs((results[i].azimuth * r2d - data[2])/data[2]));
                    }
                    if(EnableArea)
                    {
                        acc[i][2][res_index](bg::math::abs(areas[i] - data[9]));
                    }
                    //acc[i][0](data[6]);
                    //acc[i][1](bg::math::abs(results[i].distance - data[6]));
                    //acc[i][2][0](bg::math::abs((results[i].distance - data[6]) / data[6]));
                    //acc[i][3][0](bg::math::abs((results[i].azimuth * r2d - data[2]) / data[2]));
                    //acc[i][4][0](bg::math::abs((results[i].reverse_azimuth * r2d - data[5]) / data[5]));
                    //acc[i][5][0](bg::math::abs((results[i].reduced_length - data[8]) / data[8]));
                    //acc[i][6][0](bg::math::abs((areas[i] - data[9]) / data[9]));
#ifdef PRINTDATA
                    std::cout << std::setprecision(16) <<  areas[i] << " " << data[9];
                    std::cout << " e=" << bg::math::abs(areas[i] - data[9]) << "\n";
                    //std::cout << results[i].distance << " ";
                    //std::cout << " e=" << bg::math::abs(results[i].distance - data[6]) << "\n";
                    //std::cout
                    //        << bg::math::abs((results[i].distance - data[6]) / data[6])
                    //        << " "
                    //       << bg::math::abs((results[i].azimuth * r2d - data[2]) / data[2])
                    //        << " "
                    //        << bg::math::abs((results[i].reverse_azimuth * r2d - data[5]) / data[5])
                    //        << " "
                    //        << bg::math::abs((results[i].reduced_length - data[8]) / data[8])
                    //        << " "
                    //        << bg::math::abs((areas[i] - data[9]) / data[9])
                    //        << std::endl;
#endif
                }
#endif
#ifdef PRINTDATA
                std::cout << visited << ": ";
                for (size_t i = 0; i < num_of_methods; i++)
                {
                    std::cout << times[i]/visited << " ";
                }
                std::cout << std::endl;
#endif
            }//if
            count++;
        } //while
        myfile.close();

        std::cout << "#data=" << visited << "\n";

        std::cout << "Mean time per method:" << std::endl;
        for (size_t i = 0; i < num_of_methods; i++)
        {
            std::cout << times[i]/visited << std::endl;
        }

        std::cout << "#data/category" << std::endl;
        for (size_t j = 0; j < num_of_geo_categories; j++)
        {
            std::cout << ba::count(acc[0][0][j]) << " ";
        }
        std::cout << std::endl;
#ifdef STATS
        // Display statistics
        std::cout << "Mean absolute error:" << std::endl;
        for (size_t i = 0; i < num_of_methods; i++)
        {
            for (size_t j = 0; j < num_of_geo_categories; j++)
            {
                std::cout << ba::mean(acc[i][0][j]) << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
        std::cout << "Mean relative error:" << std::endl;
        for (size_t i = 0; i < num_of_methods; i++)
        {
            for (size_t j = 0; j < num_of_geo_categories; j++)
            {
                std::cout << ba::mean(acc[i][1][j]) << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
        std::cout << "Max absolute error:" << std::endl;
        for (size_t i = 0; i < num_of_methods; i++)
        {
            for (size_t j = 0; j < num_of_geo_categories; j++)
            {
                std::cout << ba::max(acc[i][0][j]) << "|";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
        std::cout << "Max relative error:" << std::endl;
        for (size_t i = 0; i < num_of_methods; i++)
        {
            for (size_t j = 0; j < num_of_geo_categories; j++)
            {
                std::cout << ba::max(acc[i][1][j]) << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
        std::cout << "Max absolute error (area):" << std::endl;
        for (size_t i = 0; i < num_of_methods; i++)
        {
            for (size_t j = 0; j < num_of_geo_categories; j++)
            {
                std::cout << ba::max(acc[i][2][j]) << "|";
            }
            std::cout << std::endl;
        }
        /*
        std::cout << "Min absolute error:" << std::endl;
        for (size_t i = 0; i < num_of_methods; i++)
        {
            for (size_t j = 0; j < num_of_geo_categories; j++)
            {
                std::cout << ba::min(acc[i][0][j]) << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
        std::cout << "Min relative error:" << std::endl;
        for (size_t i = 0; i < num_of_methods; i++)
        {
            for (size_t j = 0; j < num_of_geo_categories; j++)
            {
                std::cout << ba::min(acc[i][1][j]) << " ";
            }
            std::cout << std::endl;
        }
*/
#endif
    } //if file is open


    else std::cout << "Unable to open file";

    //}//for

    return 0;
} //main
