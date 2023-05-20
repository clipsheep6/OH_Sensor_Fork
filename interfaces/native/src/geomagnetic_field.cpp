/*
 * Copyright (c) 2021 Huawei Device Co., Ltd.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "geomagnetic_field.h"

#include <cmath>
#include <mutex>

using namespace std;
namespace {
constexpr double EARTH_MAJOR_AXIS_RADIUS = 6378.137f;
constexpr double EARTH_MINOR_AXIS_RADIUS = 6356.7523142f;
constexpr double EARTH_REFERENCE_RADIUS = 6371.2f;
constexpr double PRECISION = 1e-5f;
constexpr double LATITUDE_MAX = 90.0f;
constexpr double LATITUDE_MIN = -90.0f;
constexpr double CONVERSION_FACTOR = 1000.0f;
constexpr double DERIVATIVE_FACTOR = 1.0f;
// the time from 1970-01-01 to 2020-01-01 as UTC milliseconds from the epoch
constexpr int64_t WMM_BASE_TIME = 1580486400000;
// The following Gaussian coefficients are derived from the US/ United Kingdom World Magnetic Model 2020-2025.
constexpr double GAUSS_COEFFICIENT_G[13][13] = {
    {0.0f},
    {-29404.5f, -1450.7f},
    {-2500.0f, 2982.0f, 1676.8f},
    {1363.9f, -2381.0f, 1236.2f, 525.7f},
    {903.1f, 809.4f, 86.2f, -309.4f, 47.9f},
    {-234.4f, 363.1f, 187.8f, -140.7f, -151.2f, 13.7f},
    {65.9f, 65.6f, 73.0f, -121.5f, -36.2f, 13.5f, -64.7f},
    {80.6f, -76.8f, -8.3f, 56.5f, 15.8f, 6.4f, -7.2f, 9.8f},
    {23.6f, 9.8f, -17.5f, -0.4f, -21.1f, 15.3f, 13.7f, -16.5f, -0.3f},
    {5.0f, 8.2f, 2.9f, -1.4f, -1.1f, -13.3f, 1.1f, 8.9f, -9.3f, -11.9f},
    {-1.9f, -6.2f, -0.1f, 1.7f, -0.9f, 0.6f, -0.9f, 1.9f, 1.4f, -2.4f, -3.9f},
    {3.0f, -1.4f, -2.5f, 2.4f, -0.9f, 0.3f, -0.7f, -0.1f, 1.4f, -0.6f, 0.2f, 3.1f},
    {-2.0f, -0.1f, 0.5f, 1.3f, -1.2f, 0.7f, 0.3f, 0.5f, -0.2f, -0.5f, 0.1f, -1.1f, -0.3f}
};
constexpr double GAUSS_COEFFICIENT_H[13][13] = {
    {0.0f},
    {0.0f, 4652.9f},
    {0.0f, -2991.6f, -734.8f},
    {0.0f, -82.2f, 241.8f, -542.9f},
    {0.0f, 282.0f, -158.4f, 199.8f, -350.1f},
    {0.0f, 47.7f, 208.4f, -121.3f, 32.2f, 99.1f},
    {0.0f, -19.1f, 25.0f, 52.7f, -64.4f, 9.0f, 68.1f},
    {0.0f, -51.4f, -16.8f, 2.3f, 23.5f, -2.2f, -27.2f, -1.9f},
    {0.0f, 8.4f, -15.3f, 12.8f, -11.8f, 14.9f, 3.6f, -6.9f, 2.8f},
    {0.0f, -23.3f, 11.1f, 9.8f, -5.1f, -6.2f, 7.8f, 0.4f, -1.5f, 9.7f},
    {0.0f, 3.4f, -0.2f, 3.5f, 4.8f, -8.6f, -0.1f, -4.2f, -3.4f, -0.1f, -8.8f},
    {0.0f, 0.0f, 2.6f, -0.5f, -0.4f, 0.6f, -0.2f, -1.7f, -1.6f, -3.0f, -2.0f, -2.6f},
    {0.0f, -1.2f, 0.5f, 1.3f, -1.8f, 0.1f, 0.7f, -0.1f, 0.6f, 0.2f, -0.9f, 0.0f, 0.5f}
};
constexpr double DELTA_GAUSS_COEFFICIENT_G[13][13] = {
    {0.0f},
    {6.7f, 7.7f},
    {-11.5f, -7.1f, -2.2f},
    {2.8f, -6.2f, 3.4f, -12.2f},
    {-1.1f, -1.6f, -6.0f, 5.4f, -5.5f},
    {-0.3f, 0.6f, -0.7f, 0.1f, 1.2f, 1.0f},
    {-0.6f, -0.4f, 0.5f, 1.4f, -1.4f, 0.0f, 0.8f},
    {-0.1f, -0.3f, -0.1f, 0.7f, 0.2f, -0.5f, -0.8f, 1.0f},
    {-0.1f, 0.1f, -0.1f, 0.5f, -0.1f, 0.4f, 0.5f, 0.0f, 0.4f},
    {-0.1f, -0.2f, 0.0f, 0.4f, -0.3f, 0.0f, 0.3f, 0.0f, 0.0f, -0.4f},
    {0.0f, 0.0f, 0.0f, 0.2f, -0.1f, -0.2f, 0.0f, -0.1f, -0.2f, -0.1f, 0.0f},
    {0.0f, -0.1f, 0.0f, 0.0f, 0.0f, -0.1f, 0.0f, 0.0f, -0.1f, -0.1f, -0.1f, -0.1f},
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.1f}
};
constexpr double DELTA_GAUSS_COEFFICIENT_H[13][13] = {
    {0.0f},
    {0.0f, -25.1f},
    {0.0f, -30.2f, -23.9f},
    {0.0f, 5.7f, -1.0f, 1.1f},
    {0.0f, 0.2f, 6.9f, 3.7f, -5.6f},
    {0.0f, 0.1f, 2.5f, -0.9f, 3.0f, 0.5f},
    {0.0f, 0.1f, -1.8f, -1.4f, 0.9f, 0.1f, 1.0f},
    {0.0f, 0.5f, 0.6f, -0.7f, -0.2f, -1.2f, 0.2f, 0.3f},
    {0.0f, -0.3f, 0.7f, -0.2f, 0.5f, -0.3f, -0.5f, 0.4f, 0.1f},
    {0.0f, -0.3f, 0.2f, -0.4f, 0.4f, 0.1f, 0.0f, -0.2f, 0.5f, 0.2f},
    {0.0f, 0.0f, 0.1f, -0.3f, 0.1f, -0.2f, 0.1f, 0.0f, -0.1f, 0.2f, 0.0f},
    {0.0f, 0.0f, 0.1f, 0.0f, 0.2f, 0.0f, 0.0f, 0.1f, 0.0f, -0.1f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, -0.1f, 0.1f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, -0.1f}
};
constexpr int32_t GAUSSIAN_COEFFICIENT_DIMENSION = 13;
std::mutex mutex_;

double northComponent;
double eastComponent;
double downComponent;
double geocentricLatitude;
double geocentricLongitude;
double geocentricRadius;

std::vector<std::vector<double>> schmidtQuasiNormalFactors;
std::vector<std::vector<double>> polynomials(GAUSSIAN_COEFFICIENT_DIMENSION);
std::vector<std::vector<double>> polynomialsDerivative(GAUSSIAN_COEFFICIENT_DIMENSION);
std::vector<double> relativeRadiusPower(GAUSSIAN_COEFFICIENT_DIMENSION + 2);
std::vector<double> sinMLongitude(GAUSSIAN_COEFFICIENT_DIMENSION);
std::vector<double> cosMLongitude(GAUSSIAN_COEFFICIENT_DIMENSION);
}

GeomagneticField::GeomagneticField(double latitude, double longitude, double altitude, int64_t timeMillis)
{
    std::lock_guard<std::mutex> geomagneticLock(mutex_);
    schmidtQuasiNormalFactors = GetSchmidtQuasiNormalFactors(GAUSSIAN_COEFFICIENT_DIMENSION);
    double gcLatitude = fmax(LATITUDE_MIN + PRECISION, fmin(LATITUDE_MAX - PRECISION, latitude));
    CalibrateGeocentricCoordinates(gcLatitude, longitude, altitude);
    InitLegendreTable(GAUSSIAN_COEFFICIENT_DIMENSION - 1, static_cast<double>(M_PI / 2.0 - geocentricLatitude));
    GetRelativeRadiusPower();
    double latDiffRad = ToRadians(gcLatitude) - geocentricLatitude;
    CalculateGeomagneticComponent(latDiffRad, timeMillis);
}

std::vector<std::vector<double>> GeomagneticField::GetSchmidtQuasiNormalFactors(int32_t expansionDegree)
{
    std::vector<std::vector<double>> schmidtQuasiNormFactors(expansionDegree + 1);
    schmidtQuasiNormFactors[0].resize(1);
    schmidtQuasiNormFactors[0][0] = 1.0f;
    for (int32_t row = 1; row <= expansionDegree; row++) {
        schmidtQuasiNormFactors[row].resize(row + 1);
        schmidtQuasiNormFactors[row][0] =
            schmidtQuasiNormFactors[row - 1][0] * (2 * row - 1) / static_cast<double>(row);
        for (int32_t column = 1; column <= row; column++) {
            schmidtQuasiNormFactors[row][column] = schmidtQuasiNormFactors[row][column - 1]
                * static_cast<double>(sqrt((row - column + 1) * ((column == 1) ? 2 : 1)
                / static_cast<double>(row + column)));
        }
    }
    return schmidtQuasiNormFactors;
}

void GeomagneticField::CalculateGeomagneticComponent(double latDiffRad, int64_t timeMillis)
{
    double yearsSinceBase = (timeMillis - WMM_BASE_TIME) / (365.0f * 24.0f * 60.0f * 60.0f * 1000.0f);
    double inverseCosLatitude = DERIVATIVE_FACTOR / static_cast<double>(cos(geocentricLatitude));
    GetLongitudeTrigonometric();
    double gcX = 0.0f;
    double gcY = 0.0f;
    double gcZ = 0.0f;
    for (int32_t row = 1; row < GAUSSIAN_COEFFICIENT_DIMENSION; row++) {
        for (int32_t column = 0; column <= row; column++) {
            double g = GAUSS_COEFFICIENT_G[row][column] + yearsSinceBase
                * DELTA_GAUSS_COEFFICIENT_G[row][column];
            double h = GAUSS_COEFFICIENT_H[row][column] + yearsSinceBase
                * DELTA_GAUSS_COEFFICIENT_H[row][column];
            gcX += relativeRadiusPower[row + 2]
                * (g * cosMLongitude[column] + h * sinMLongitude[column])
                * polynomialsDerivative[row][column]
                * schmidtQuasiNormalFactors[row][column];
            gcY += relativeRadiusPower[row + 2] * column
                * (g * sinMLongitude[column] - h * cosMLongitude[column])
                * polynomials[row][column]
                * schmidtQuasiNormalFactors[row][column]
                * inverseCosLatitude;
            gcZ -= (row + 1) * relativeRadiusPower[row + 2]
                * (g * cosMLongitude[column] + h * sinMLongitude[column])
                * polynomials[row][column]
                * schmidtQuasiNormalFactors[row][column];
        }
        northComponent = static_cast<double>(gcX * cos(latDiffRad) + gcZ * sin(latDiffRad));
        eastComponent = gcY;
        downComponent = static_cast<double>(-gcX * sin(latDiffRad) + gcZ * cos(latDiffRad));
    }
}

void GeomagneticField::GetLongitudeTrigonometric()
{
    sinMLongitude[0] = 0.0f;
    cosMLongitude[0] = 1.0f;
    sinMLongitude[1] = static_cast<double>(sin(geocentricLongitude));
    cosMLongitude[1] = static_cast<double>(cos(geocentricLongitude));
    for (uint32_t index = 2; index < GAUSSIAN_COEFFICIENT_DIMENSION; ++index) {
        uint32_t x = index >> 1;
        sinMLongitude[index] = (sinMLongitude[index - x] * cosMLongitude[x]
            + cosMLongitude[index - x] * sinMLongitude[x]);
        cosMLongitude[index] = (cosMLongitude[index - x] * cosMLongitude[x]
            - sinMLongitude[index - x] * sinMLongitude[x]);
    }
}

void GeomagneticField::GetRelativeRadiusPower()
{
    relativeRadiusPower[0] = 1.0f;
    relativeRadiusPower[1] = EARTH_REFERENCE_RADIUS / geocentricRadius;
    for (int32_t index = 2; index < static_cast<int32_t>(relativeRadiusPower.size()); ++index) {
        relativeRadiusPower[index] = relativeRadiusPower[index - 1] * relativeRadiusPower[1];
    }
}

void GeomagneticField::CalibrateGeocentricCoordinates(double latitude, double longitude, double altitude)
{
    double altitudeKm = altitude / CONVERSION_FACTOR;
    double a2 = EARTH_MAJOR_AXIS_RADIUS * EARTH_MAJOR_AXIS_RADIUS;
    double b2 = EARTH_MINOR_AXIS_RADIUS * EARTH_MINOR_AXIS_RADIUS;
    double gdLatRad = ToRadians(latitude);
    double clat = static_cast<double>(cos(gdLatRad));
    double slat = static_cast<double>(sin(gdLatRad));
    double tlat = slat / clat;
    double latRad = static_cast<double>(sqrt(a2 * clat * clat + b2 * slat * slat));
    geocentricLatitude = static_cast<double>(atan(tlat * (latRad * altitudeKm + b2)
        / (latRad * altitudeKm + a2)));
    geocentricLongitude = static_cast<double>(ToRadians(longitude));
    double radSq = altitudeKm * altitudeKm + 2 * altitudeKm
        * latRad + (a2 * a2 * clat * clat + b2 * b2 * slat * slat)
        / (a2 * clat * clat + b2 * slat * slat);
    geocentricRadius = static_cast<double>(sqrt(radSq));
}

void GeomagneticField::InitLegendreTable(int32_t expansionDegree, double thetaRad)
{
    polynomials[0].resize(1);
    polynomials[0][0] = 1.0f;
    polynomialsDerivative[0].resize(1);
    polynomialsDerivative[0][0] = 0.0f;
    double cosValue = static_cast<double>(cos(thetaRad));
    double sinValue = static_cast<double>(sin(thetaRad));
    for (int32_t row = 1; row <= expansionDegree; row++) {
        polynomials[row].resize(row + 1);
        polynomialsDerivative[row].resize(row + 1);
        for (int32_t column = 0; column <= row; column++) {
            if (row == column) {
                polynomials[row][column] = sinValue * polynomials[row - 1][column - 1];
                polynomialsDerivative[row][column] = cosValue * polynomials[row - 1][column - 1]
                    + sinValue * polynomialsDerivative[row - 1][column - 1];
            } else if (row == 1 || column == row - 1) {
                polynomials[row][column] = cosValue * polynomials[row - 1][column];
                polynomialsDerivative[row][column] = -sinValue * polynomials[row - 1][column]
                    + cosValue * polynomialsDerivative[row - 1][column];
            } else {
                double k = ((row - 1) * (row - 1) - column * column)
                    / static_cast<double>((2 * row - 1) * (2 * row - 3));
                polynomials[row][column] = cosValue * polynomials[row - 1][column]
                    - k * polynomials[row - 2][column];
                polynomialsDerivative[row][column] = -sinValue * polynomials[row - 1][column]
                    + cosValue * polynomialsDerivative[row - 1][column]
                    - k * polynomialsDerivative[row - 2][column];
            }
        }
    }
}

double GeomagneticField::ObtainX()
{
    std::lock_guard<std::mutex> geomagneticLock(mutex_);
    return northComponent;
}

double GeomagneticField::ObtainY()
{
    std::lock_guard<std::mutex> geomagneticLock(mutex_);
    return eastComponent;
}

double GeomagneticField::ObtainZ()
{
    std::lock_guard<std::mutex> geomagneticLock(mutex_);
    return downComponent;
}

double GeomagneticField::ObtainGeomagneticDip()
{
    std::lock_guard<std::mutex> geomagneticLock(mutex_);
    double horizontalIntensity = hypot(northComponent, eastComponent);
    return static_cast<double>(ToDegrees(atan2(downComponent, horizontalIntensity)));
}

double GeomagneticField::ToDegrees(double angrad)
{
    return angrad * 180.0 / M_PI;
}

double GeomagneticField::ToRadians(double angdeg)
{
    return angdeg / 180.0 * M_PI;
}

double GeomagneticField::ObtainDeflectionAngle()
{
    std::lock_guard<std::mutex> geomagneticLock(mutex_);
    return static_cast<double>(ToDegrees(atan2(eastComponent, northComponent)));
}

double GeomagneticField::ObtainLevelIntensity()
{
    std::lock_guard<std::mutex> geomagneticLock(mutex_);
    double horizontalIntensity = hypot(northComponent, eastComponent);
    return horizontalIntensity;
}

double GeomagneticField::ObtainTotalIntensity()
{
    std::lock_guard<std::mutex> geomagneticLock(mutex_);
    double sumOfSquares = northComponent * northComponent + eastComponent * eastComponent
        + downComponent * downComponent;
    double totalIntensity = static_cast<double>(sqrt(sumOfSquares));
    return totalIntensity;
}

