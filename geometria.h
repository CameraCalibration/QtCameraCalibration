#ifndef GEOMETRIA_H
#define GEOMETRIA_H

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <opencv2/opencv.hpp>

#define EPS 1e-8
#define PI acos(-1)
#define Vector PointFrz

struct PointFrz
{
    double x, y;
    PointFrz(){}
    PointFrz(double a, double b) { x = a; y = b; }
    double mod2() { return x*x + y*y; }
    double mod()  { return sqrt(x*x + y*y); }
    double arg()  { return atan2(y, x); }
    PointFrz ort()   { return PointFrz(-y, x); }
    PointFrz unit()  { double k = mod(); return PointFrz(x/k, y/k); }
};

PointFrz operator +(const PointFrz &a, const PointFrz &b) { return PointFrz(a.x + b.x, a.y + b.y); }
PointFrz operator -(const PointFrz &a, const PointFrz &b) { return PointFrz(a.x - b.x, a.y - b.y); }
PointFrz operator /(const PointFrz &a, double k) { return PointFrz(a.x/k, a.y/k); }
PointFrz operator *(const PointFrz &a, double k) { return PointFrz(a.x*k, a.y*k); }

bool operator ==(const PointFrz &a, const PointFrz &b)
{
    return fabs(a.x - b.x) < EPS && fabs(a.y - b.y) < EPS;
}
bool operator !=(const PointFrz &a, const PointFrz &b)
{
    return !(a==b);
}
bool operator <(const PointFrz &a, const PointFrz &b)
{
    if(a.x != b.x) return a.x < b.x;
    return a.y < b.y;
}

//### FUNCIONES BASICAS #############################################################

double dist(const PointFrz &A, const PointFrz &B)    { return hypot(A.x - B.x, A.y - B.y); }
double cross(const Vector &A, const Vector &B) { return A.x * B.y - A.y * B.x; }
double dot(const Vector &A, const Vector &B)   { return A.x * B.x + A.y * B.y; }
double area(const PointFrz &A, const PointFrz &B, const PointFrz &C) { return cross(B - A, C - A); }

// Heron triangulo y cuadrilatero ciclico
// http://mathworld.wolfram.com/CyclicQuadrilateral.html
// http://www.spoj.pl/problems/QUADAREA/

double areaHeron(double a, double b, double c)
{
    double s = (a + b + c) / 2;
    return sqrt(s * (s-a) * (s-b) * (s-c));
}

double circumradius(double a, double b, double c) { return a * b * c / (4 * areaHeron(a, b, c)); }

double areaHeron(double a, double b, double c, double d)
{
    double s = (a + b + c + d) / 2;
    return sqrt((s-a) * (s-b) * (s-c) * (s-d));
}

double circumradius(double a, double b, double c, double d) { return sqrt((a*b + c*d) * (a*c + b*d) * (a*d + b*c))  / (4 * areaHeron(a, b, c, d)); }

//### DETERMINA SI P PERTENECE AL SEGMENTO AB ###########################################
bool onSegment(const PointFrz &A, const PointFrz &B, const PointFrz &P)
{
    return abs(area(A, B, P)) < EPS &&
            P.x >= std::min(A.x, B.x) && P.x <= std::max(A.x, B.x) &&
            P.y >= std::min(A.y, B.y) && P.y <= std::max(A.y, B.y);
}

//### DETERMINA SI EL SEGMENTO P1Q1 SE INTERSECTA CON EL SEGMENTO P2Q2 #####################
bool intersects(const PointFrz &P1, const PointFrz &P2, const PointFrz &P3, const PointFrz &P4)
{
    double A1 = area(P3, P4, P1);
    double A2 = area(P3, P4, P2);
    double A3 = area(P1, P2, P3);
    double A4 = area(P1, P2, P4);

    if( ((A1 > 0 && A2 < 0) || (A1 < 0 && A2 > 0)) &&
        ((A3 > 0 && A4 < 0) || (A3 < 0 && A4 > 0)))
            return true;

    else if(A1 == 0 && onSegment(P3, P4, P1)) return true;
    else if(A2 == 0 && onSegment(P3, P4, P2)) return true;
    else if(A3 == 0 && onSegment(P1, P2, P3)) return true;
    else if(A4 == 0 && onSegment(P1, P2, P4)) return true;
    else return false;
}

//### DETERMINA SI A, B, M, N PERTENECEN A LA MISMA RECTA ##############################
bool sameLine(PointFrz P1, PointFrz P2, PointFrz P3, PointFrz P4)
{
    return area(P1, P2, P3) == 0 && area(P1, P2, P4) == 0;
}
//### SI DOS SEGMENTOS O RECTAS SON PARALELOS ###################################################
bool isParallel(const PointFrz &P1, const PointFrz &P2, const PointFrz &P3, const PointFrz &P4)
{
    return cross(P2 - P1, P4 - P3) == 0;
}

//### PUNTO DE INTERSECCION DE DOS RECTAS NO PARALELAS #################################
PointFrz lineIntersection(const PointFrz &A, const PointFrz &B, const PointFrz &C, const PointFrz &D)
{
    return A + (B - A) * (cross(C - A, D - C) / cross(B - A, D - C));
}

//### FUNCIONES BASICAS DE POLIGONOS ################################################
bool isConvex(const std::vector <PointFrz> &P)
{
    int n = P.size(), pos = 0, neg = 0;
    for(int i=0; i<n; i++)
    {
        double A = area(P[i], P[(i+1)%n], P[(i+2)%n]);
        if(A < 0) neg++;
        else if(A > 0) pos++;
    }
    return neg == 0 || pos == 0;
}

double area(const std::vector <PointFrz> &P)
{
    int n = P.size();
    double A = 0;
    for(int i=1; i<=n-2; i++)
        A += area(P[0], P[i], P[i+1]);
    return abs(A/2);
}

bool PointFrzInPoly(const std::vector <PointFrz> &P, const PointFrz &A)
{
    int n = P.size(), cnt = 0;
    for(int i=0; i<n; i++)
    {
        int inf = i, sup = (i+1)%n;
        if(P[inf].y > P[sup].y) std::swap(inf, sup);
        if(P[inf].y <= A.y && A.y < P[sup].y)
            if(area(A, P[inf], P[sup]) > 0)
                cnt++;
    }
    return (cnt % 2) == 1;
}

//### CONVEX HULL ######################################################################

// O(n log n)
/*vector <PointFrz> ConvexHull(vector <PointFrz> P)
{
    sort(P.begin(),P.end());
    int n = P.size(),k = 0;
//    PointFrz H[2*n];
    PointFrz *H;
    H = (PointFrz *)malloc(sizeof(PointFrz)*2*n);

    for(int i=0;i<n;++i){
        while(k>=2 && area(H[k-2],H[k-1],P[i]) <= 0) --k;
        H[k++] = P[i];
    }

    for(int i=n-2,t=k;i>=0;--i){
        while(k>t && area(H[k-2],H[k-1],P[i]) <= 0) --k;
        H[k++] = P[i];
    }
    free(H);

    return vector <PointFrz> (H,H+k-1);
}*/

//### DETERMINA SI P ESTA EN EL INTERIOR DEL POLIGONO CONVEXO A ########################

bool isInConvexSlow(const std::vector <PointFrz> &P, const PointFrz &A)
{
    int n = P.size(), pos = 0, neg = 0;
    for(int i=0; i<n; i++)
    {
        double AA = area(A, P[i], P[(i+1)%n]);
        if(AA < 0) neg++;
        else if(AA > 0) pos++;
    }
    return neg == 0 || pos == 0;
}

// O (log n)
bool isInConvex(const std::vector <PointFrz> &A, const PointFrz &P)
{
    int n = A.size(), lo = 1, hi = A.size() - 1;

    if(area(A[0], A[1], P) <= 0) return 0;
    if(area(A[n-1], A[0], P) <= 0) return 0;

    while(hi - lo > 1)
    {
        int mid = (lo + hi) / 2;

        if(area(A[0], A[mid], P) > 0) lo = mid;
        else hi = mid;
    }

    return area(A[lo], A[hi], P) > 0;
}

// O(n)
PointFrz norm(const PointFrz &A, const PointFrz &O)
{
    Vector V = A - O;
    V = V * 10000000000.0 / V.mod();
    return O + V;
}

bool isInConvex(std::vector <PointFrz> &A, std::vector <PointFrz> &B)
{
    if(!isInConvex(A, B[0])) return 0;
    else
    {
        int n = A.size(), p = 0;

        for(int i=1; i<B.size(); i++)
        {
            while(!intersects(A[p], A[(p+1)%n], norm(B[i], B[0]), B[0])) p = (p+1)%n;

            if(area(A[p], A[(p+1)%n], B[i]) <= 0) return 0;
        }

        return 1;
    }
}

//##### SMALLEST ENCLOSING CIRCLE ########################################################

PointFrz circumcenter(const PointFrz &A, const PointFrz &B, const PointFrz &C)
{
    PointFrz M1 = (A + B) / 2;
    PointFrz M2 = (A + C) / 2;
    return lineIntersection(M1, M1 + (A - B).ort(), M2, M2 + (A - C).ort());
}

std::pair <PointFrz, double> enclosingCircle(std::vector <PointFrz> P)
{
    random_shuffle(P.begin(), P.end());

    PointFrz O(0, 0);
    double R2 = 0;

    for(int i=0; i<P.size(); i++)
    {
        if((P[i] - O).mod2() > R2 + EPS)
        {
            O = P[i], R2 = 0;
            for(int j=0; j<i; j++)
            {
                if((P[j] - O).mod2() > R2 + EPS)
                {
                    O = (P[i] + P[j])/2, R2 = (P[i] - P[j]).mod2() / 4;
                    for(int k=0; k<j; k++)
                        if((P[k] - O).mod2() > R2 + EPS)
                            O = circumcenter(P[i], P[j], P[k]), R2 = (P[k] - O).mod2();
                }
            }
        }
    }
    return std::make_pair(O, sqrt(R2));
}

//##### CLOSEST PAIR OF PointFrzS ########################################################
bool XYorder(PointFrz P1, PointFrz P2)
{
    if(P1.x != P2.x) return P1.x < P2.x;
    return P1.y < P2.y;
}
bool YXorder(PointFrz P1, PointFrz P2)
{
    if(P1.y != P2.y) return P1.y < P2.y;
    return P1.x < P2.x;
}
double closest_recursive(std::vector <PointFrz> vx, std::vector <PointFrz> vy)
{
    if(vx.size()==1) return 1e20;
    if(vx.size()==2) return dist(vx[0], vx[1]);

    PointFrz cut = vx[vx.size()/2];

    std::vector <PointFrz> vxL, vxR;
    for(int i=0; i<vx.size(); i++)
        if(vx[i].x < cut.x || (vx[i].x == cut.x && vx[i].y <= cut.y))
            vxL.push_back(vx[i]);
        else vxR.push_back(vx[i]);

    std::vector <PointFrz> vyL, vyR;
    for(int i=0; i<vy.size(); i++)
        if(vy[i].x < cut.x || (vy[i].x == cut.x && vy[i].y <= cut.y))
            vyL.push_back(vy[i]);
        else vyR.push_back(vy[i]);

    double dL = closest_recursive(vxL, vyL);
    double dR = closest_recursive(vxR, vyR);
    double d = std::min(dL, dR);

    std::vector <PointFrz> b;
    for(int i=0; i<vy.size(); i++)
        if(abs(vy[i].x - cut.x) <= d)
            b.push_back(vy[i]);

    for(int i=0; i<b.size(); i++)
        for(int j=i+1; j<b.size() && (b[j].y - b[i].y) <= d; j++)
            d = std::min(d, dist(b[i], b[j]));

    return d;
}
double closest(std::vector <PointFrz> PointFrzs)
{
    std::vector <PointFrz> vx = PointFrzs, vy = PointFrzs;
    std::sort(vx.begin(), vx.end(), XYorder);
    std::sort(vy.begin(), vy.end(), YXorder);

    for(int i=0; i+1<vx.size(); i++)
        if(vx[i] == vx[i+1])
            return 0.0;

    return closest_recursive(vx,vy);
}

struct StruSegme{
    std::vector<std::pair<float,float> > vectorPoint;
    std::pair<float,float> direc;
};

bool compareUpToDown(const std::pair<int,int>&i, const std::pair<int,int>&j){
    return i.second < j.second;
}

bool compareDownToUp(const std::pair<int,int>&i, const std::pair<int,int>&j){
    return i.second > j.second;
}

bool compareLeftToRight(const std::pair<int,int>&i, const std::pair<int,int>&j){
    return i.first < j.first;
}

bool compareRightToLeft(const std::pair<int,int>&i, const std::pair<int,int>&j){
    return i.first > j.first;
}

bool compareBySecond(const std::pair<int,int>&i, const std::pair<int,int>&j){
    return i.second < j.second;
}

bool compareSortColumsLeftToRight(const StruSegme&i, const StruSegme&j){
    return i.vectorPoint[0].first < j.vectorPoint[0].first;
}

bool compareSortColumsRightToLeft(const StruSegme&i, const StruSegme&j){
    return i.vectorPoint[0].first > j.vectorPoint[0].first;
}

bool compareSortRowsUpToDown(const StruSegme&i, const StruSegme&j){
    return i.vectorPoint[0].second < j.vectorPoint[0].second;
}

bool compareSortRowsDownToUp(const StruSegme&i, const StruSegme&j){
    return i.vectorPoint[0].second > j.vectorPoint[0].second;
}

bool compareDistWithPoint(const std::pair<double, std::pair<int,int> >&i, const std::pair<double, std::pair<int,int> >&j){
    return i.first < j.first;
}

bool compareByDist(const std::pair<double,int>&i, const std::pair<double,int>&j){
    return i.first < j.first;
}

std::vector<std::pair<float,float> > sortUpToDown(std::vector<std::pair<float,float> > array){
    sort(array.begin(), array.end(), compareUpToDown);
    return array;
}

std::vector<std::pair<float,float> > sortDownToUp(std::vector<std::pair<float,float> > array){
    sort(array.begin(), array.end(), compareDownToUp);
    return array;
}

std::vector<std::pair<float,float> > sortLeftToRight(std::vector<std::pair<float, float> > array){
    sort(array.begin(), array.end(), compareLeftToRight);
    return array;
}

std::vector<std::pair<float,float> > sortRightToLeft(std::vector<std::pair<float, float> > array){
    sort(array.begin(), array.end(), compareRightToLeft);
    return array;
}

std::vector<StruSegme> sortColumnsLeftToRight(std::vector<StruSegme> array){
    std::sort(array.begin(), array.end(), compareSortColumsLeftToRight);
    return array;
}

std::vector<StruSegme> sortColumnsRightToLeft(std::vector<StruSegme> array){
    sort(array.begin(), array.end(), compareSortColumsRightToLeft);
    return array;
}

std::vector<StruSegme> sortRowsUpToDown(std::vector<StruSegme> array){
    sort(array.begin(), array.end(), compareSortRowsUpToDown);
    return array;
}

std::vector<StruSegme> sortRowsDownToUp(std::vector<StruSegme> array){
    sort(array.begin(), array.end(), compareSortRowsDownToUp);
    return array;
}

std::vector<std::pair<int,float> > sortBySecond(std::vector<std::pair<int,float> > array){
    sort(array.begin(), array.end(), compareUpToDown);
    return array;
}

cv::Point2f getCentroide(std::vector<cv::Point2f> array){
    float x = 0, y = 0;
    int n = array.size();
    for(int i = 0; i < n; i++){
        x += array[i].x;
        y += array[i].y;
    }
    return cv::Point2f(x/n, y/n);
}

std::vector<std::pair<int,int> > transformVectorPointToPair(std::vector<cv::Point> array){
    std::vector<std::pair<int,int> > res;
    for(int i = 0; i < (int)array.size(); i++)
        res.push_back(std::make_pair(array[i].x, array[i].y));
    return res;
}

std::vector<StruSegme> getRowsByGridUsingPendents(std::vector<cv::Point> array, int numCols, int numRows, float minDist){
    std::vector<std::pair<double,double> > auxCol;
    std::set<std::vector<std::pair<double, double> > > columns2;
    double distancia, distancia2;

    // Obtencion de todas las rectas que tienen el mismo numero de array en la recta (numRows)

    PointFrz A, B, P;
    for(size_t i = 0; i < array.size(); i++){
        for(size_t j = i+1; j < array.size(); j++){
            A.x = array[i].x;
            A.y = array[i].y;
            B.x = array[j].x;
            B.y = array[j].y;
            auxCol.clear();
            for(size_t k = 0; k < array.size(); k++){
                if(k == i || k == j) continue;
                P.x = array[k].x;
                P.y = array[k].y;
                distancia = cross((P - A), (B - A)) / (B - A).mod();
                distancia2 = cross((P - B), (A - B)) / (A - B).mod();

                if(abs(distancia) <= minDist && abs(distancia2) <= minDist) {
                    auxCol.push_back(std::make_pair(P.x, P.y));
                }
            }
            if((int)auxCol.size() == (numRows - 2)){
                auxCol.push_back(std::make_pair(array[i].x, array[i].y));
                auxCol.push_back(std::make_pair(array[j].x, array[j].y));
                sort(auxCol.begin(), auxCol.end());
                columns2.insert(auxCol);
            }
        }
    }

    // Colocamos las columnas en un vector
    std::vector<std::vector<std::pair<double, double> > > rectas;
    for (std::set<std::vector<std::pair<double,double> > >::iterator it = columns2.begin(); it != columns2.end(); it++){
        rectas.push_back((*it));
    }

    // Calculamos la pendiente de todas las rectas
    double pendiente = 0;
    std::vector<std::pair<double,int> > freq;
    for (int i=0; i<(int)rectas.size(); i++){
        if((rectas[i][0].first - rectas[i][1].first) == 0)
            pendiente = 10000;
        else
            pendiente = (rectas[i][0].second - rectas[i][1].second) * 1.0 / (rectas[i][0].first - rectas[i][1].first);
        freq.push_back(std::make_pair(pendiente, i));
    }

    sort(freq.begin(), freq.end());
    double dist, diffPen = 0.25;
    std::vector<std::vector<std::pair<double, double> > > rectSelected;
    rectSelected.push_back(rectas[freq[0].second]);
    for(int i=1; i < (int)freq.size(); i++)
    {
        dist = abs(freq[i].first-freq[i-1].first);
        if(dist < diffPen) {
             rectSelected.push_back(rectas[freq[i].second]);
             if((int)rectSelected.size() == numCols)
                 break;
        }
        else {
            if((int)rectSelected.size() < numCols) {
                rectSelected.clear();
                 rectSelected.push_back(rectas[freq[i].second]);
            }
            else
                break;
        }
    }

    std::vector<StruSegme> res;
    for(int i = 0; i < (int)rectSelected.size(); i++){
        StruSegme tmp;
        std::vector<std::pair<float,float> > vecPoin;
        for(int j = 0; j < numRows; j++){
            float x = rectSelected[i][j].first;
            float y = rectSelected[i][j].second;
            vecPoin.push_back(std::make_pair(x,y));
        }
        tmp.vectorPoint = vecPoin;
        res.push_back(tmp);
    }
    return res;
}


std::vector<int> getPosCornes(std::vector<std::vector<cv::Point2f> > hull){
    std::vector<int> posCornes;
    double piValue = 3.14159265, maxAngle = 148;
    int nHull = (int)hull[0].size();
    for(int i = 0; i < nHull; i++){
        float angle;
        PointFrz A(hull[0][i].x - hull[0][(i==0?nHull:i)-1].x, hull[0][i].y - hull[0][(i==0?nHull:i)-1].y);
        PointFrz B(hull[0][i].x - hull[0][i==nHull-1?0:i+1].x, hull[0][i].y - hull[0][i==nHull-1?0:i+1].y);
        angle = atan2(cross(A,B),dot(A,B)) * 180 / piValue;
        if(abs(angle) <= maxAngle)
            posCornes.push_back(i);
    }
    return posCornes;
}

std::vector<std::pair<double, std::pair<int,int> > > sortByDistWithPoint(std::vector<std::pair<double, std::pair<int,int> > > array){
    sort(array.begin(), array.end(), compareDistWithPoint);
    return array;
}

std::vector<std::pair<double,int> > sortRectVSPoint(std::vector<std::pair<double,int> > array, int pos){
    sort(array.begin() + pos, array.end(), compareByDist);
    return array;
}

double getAnglesBetweenRects(cv::Point a, cv::Point b){
    PointFrz A(a.x,a.y);
    PointFrz B(b.x,b.y);
    double piValue = 3.14159265;
    return atan2(cross(A,B),dot(A,B)) * 180 / piValue;
}

double getDistanPointToRect(cv::Point a,cv::Point b,cv::Point c){
    PointFrz A(a.x,a.y);
    PointFrz B(b.x,b.y);
    PointFrz P(c.x,c.y);
    return cross((P - A), (B - A)) / (B - A).mod();
}


#endif // GEOMETRIA_H
