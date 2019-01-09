#ifndef STATISTICS_H
#define STATISTICS_H

#include <iostream>
#include <math.h>
#include <vector>
#include <algorithm>

#define ERRORF          0.0001
#define AUTO_INTERVAL   -1

using namespace std;

namespace metrics {

///
/// \brief getFrequences    Funcion que calcula las frecuencias de un conjunto de datos
/// \param elems        Elementos que seran evaluados para obtener sus frecuencias (valor_a_evaluar, id_aux)
/// \param freqs        Vector que almacenara las frecuencias de cada intervalo (frecuencia, etiqueta_intervalo)
/// \param extraInfo    Vector que almacenara informacion adicional ((minInterval, maxInterval), (posIniMinInterval, posFinMaxInterval))
/// \param asc          Booleano que indica si se ordenara de forma ascendente o descendente
/// \param nIntervals   Numero de intervalos que se desea generar, si se coloca el valor AUTO_INTERVAL se calculara automaticamente este valor
///
template <typename T, typename U>
void getFrequences(vector<pair<T,U> > &elems, vector<pair<int, float> > &freqs, vector<pair<pair<float,float>, pair<int,int> > > &extraInfo, bool asc, int nIntervals=AUTO_INTERVAL)
{
    if(nIntervals == AUTO_INTERVAL || nIntervals < 1) {
        nIntervals = 1 + 3.332 * log10(elems.size());
        if(nIntervals % 2 == 0)
            nIntervals++;
    }

//    cout << "NroDatos: " << elems.size() << endl;
//    cout << "NroIntervals: " << nIntervals << endl;

    float amplitude, minVal, maxVal;

    if(asc) {
        sort(elems.begin(), elems.end());
        int cont = 0, iniInterval = 0;
        amplitude = abs(elems[0].first - elems[elems.size()-1].first) * 1.0 / nIntervals;
        minVal = elems[0].first;
        maxVal = minVal + amplitude;
        int i;
        for(i = 0; i < (int)elems.size(); i++) {
            if (elems[i].first >= (minVal - ERRORF) && elems[i].first <= (maxVal + ERRORF))
                cont++;
            else {
                freqs.push_back(make_pair(cont, (minVal + maxVal) * 0.5));
                extraInfo.push_back(make_pair(make_pair(minVal, maxVal), make_pair(iniInterval, i)));
                iniInterval = i;
                cont = 0;
                i++;
                minVal = maxVal;
                maxVal = maxVal + amplitude;
            }
        }
        // Guardando el conteo del ultimo intervalo
        if(cont > 0) {
            freqs.push_back(make_pair(cont, (minVal + maxVal) * 0.5));
            extraInfo.push_back(make_pair(make_pair(minVal, maxVal), make_pair(iniInterval, i)));
        }
    }
    else {
        sort(elems.rbegin(), elems.rend());
        int cont = 0, iniInterval = elems.size()-1;
        amplitude = abs(elems[0].first - elems[elems.size()-1].first) * 1.0 / nIntervals;
        minVal = elems[elems.size()-1].first;
        maxVal = minVal + amplitude;
        int i;
        for(i = elems.size() - 1; i >= 0; i--) {
            if (elems[i].first >= (minVal - ERRORF) && elems[i].first <= (maxVal + ERRORF))
                cont++;
            else {
                freqs.push_back(make_pair(cont, (minVal + maxVal) * 0.5));
                extraInfo.push_back(make_pair(make_pair(minVal, maxVal), make_pair(iniInterval, i)));
                iniInterval = i;
                cont = 0;
                i++;
                minVal = maxVal;
                maxVal = maxVal + amplitude;
            }
        }
        // Guardando el conteo del ultimo intervalo
        if(cont > 0) {
            freqs.push_back(make_pair(cont, (minVal + maxVal) * 0.5));
            extraInfo.push_back(make_pair(make_pair(minVal, maxVal), make_pair(iniInterval, i)));
        }
    }
//    for(size_t i = 0; i < freqs.size(); i++) {
//        cout << "(" << extraInfo[i].first.first << "(" << extraInfo[i].second.first << ") - " << extraInfo[i].first.second << "(" << extraInfo[i].second.second << ")] ( "
//             << freqs[i].second << " ) => " << freqs[i].first << endl;
//    }
}

///
/// \brief getAvgStd    Funcion que calcula el promedio y desviacion estandar de la tabla de frecuencias dada
/// \param freqs    Tabla de frecuencias que contiene el resumen de los datos
/// \param avg      Valor promedio del conjunto de datos
/// \param std      Desviacion estandar del conjunto de datos
///
void getAvgStd(vector<pair<int, float> > freqs, float &avg, float &std)
{
    int elemTot = 0;
    avg = 0;
    for(size_t i = 0; i < freqs.size(); i++) {
        avg += freqs[i].second * freqs[i].first;
        elemTot += freqs[i].first;
    }
    avg = avg / elemTot;

    std = 0;
    for(size_t i = 0; i < freqs.size(); i++) {
        std += (freqs[i].second - avg) * (freqs[i].second - avg) * freqs[i].first;
    }
    std = sqrt(std / (elemTot - 1));
//    cout << "TotElem:" << elemTot << " -> AVG: " << avg << " -> STD: " << std << endl;
}

///
/// \brief getMode      Funcion que obtiene la moda y el intervalo donde se encuentra
/// \param freqs        Tabla de frecuencias que contiene el resumen de los datos
/// \param valMode      Valor moda del conjunto de datos
/// \param idInterval   Intervalo donde se encuentra el valor de moda
///
void getMode(vector<pair<int, float> > freqs, int &valMode, int &idInterval)
{
    valMode = 0;
    for(size_t i = 0; i < freqs.size(); i++) {
        if(freqs[i].first > valMode) {
            valMode = freqs[i].first;
            idInterval = i;
        }
    }
//    cout << "MODE: " << valMode << " - PosMode: " << idInterval << endl;
}

}

#endif // STATISTICS_H
