#include "pf.h"

PFRobot::PFRobot() {

}

void PFRobot::particleFilter(reading s, command c) {
    vector <particle> newBel(MIN_PARTICLES);
    int 
        numParticles = bel.size(),
        m, i = 0;
    double 
        sumWeight = 0, 
        *cumulativeDensityFunction = new double[numParticles],
        trial, 
        currentWeight;
    // apply motion update and assign weight based on sensor update
    for (m=0; m<numParticles; m++) {
        particle x = bel.pop_back();
        motion_update(c, &x);
        sensor_update(s, &x);
        sumWeight += x.weight;
        cumulativeDensityFunction[m] = sumWeight;
        newBel.push_back(x);
    }

    // resampling. Complexity O(MIN_PARTICLES + numParticles).
    currentWeight = newBel[0].weight;
    for (m=0; m<MIN_PARTICLES; m++) {
        trial = sumWeight * (1 - pow(random(), 1.0 / (MIN_PARTICLES - m)));
        sumWeight -= trial;
        while(trial > currentWeight) {
            trial -= currentWeight;
            i++;
            currentWeight = newBel[i].weight;
        }
        currentWeight -= trial;
        bel.push_back(newBel[i]);
    }
    newBel.clear();
}

void PFRobot::motionUpdate(command c, particle * p) {
}

void PFRobot::sensorUpdate(reading s, particle * p) {
    p->weight; // set to probability particle p produces sensor reading s
}
