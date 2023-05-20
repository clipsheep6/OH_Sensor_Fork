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

#ifndef GEOMAGNETIC_FIELD_H
#define GEOMAGNETIC_FIELD_H

#include <cstdint>
#include <vector>

class GeomagneticField {
public:
    GeomagneticField(double latitude, double longitude, double altitude, int64_t timeMillis);
    ~GeomagneticField() = default;
    double ObtainX();
    double ObtainY();
    double ObtainZ();
    double ObtainGeomagneticDip();
    double ObtainDeflectionAngle();
    double ObtainLevelIntensity();
    double ObtainTotalIntensity();

private:
    std::vector<std::vector<double>> GetSchmidtQuasiNormalFactors(int32_t expansionDegree);
    void CalculateGeomagneticComponent(double latDiffRad, int64_t timeMillis);
    void GetLongitudeTrigonometric();
    void GetRelativeRadiusPower();
    void CalibrateGeocentricCoordinates(double latitude, double longitude, double altitude);
    void InitLegendreTable(int32_t expansionDegree, double thetaRad);
    double ToDegrees(double angrad);
    double ToRadians(double angdeg);
};
#endif // GEOMAGNETIC_FIELD_H