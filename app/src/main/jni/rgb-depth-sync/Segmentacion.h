//
// Created by Adrian on 10/12/2017.
//
#pragma once
#ifndef PRUEBA_CPP_SEGMENTACION_H
#define PRUEBA_CPP_SEGMENTACION_H

#include <tango_client_api.h>
#include <tango-gl/util.h>
#include <math.h>
#include <vector>
#include <map>
#include <set>
#include <random>
#include "rgb-depth-sync/Elemento3D.h"
#include "rgb-depth-sync/Plano3D.h"
#include "rgb-depth-sync/Punto3D.h"

using namespace std;

const float multiplicadorTamaño = 10;
const float multiplicadorPosicion = 4;
const float multiplicadorDistancia = 4;
const int nRelevantes = 3;
const float umbralProf1 = 0.01;
const float umbralProf2 = 0.005;
const int minPuntos = 250;
const int minPuntosDepth = 300;
//const float distNormal = 1; //0 grados
//const float distNormal = 0.9998477; //1 grados
//const float distNormal = 0.99939083; //2 grados
//const float distNormal = 0.99862953; //3 grados
const float distNormal = 0.99756405; //4 grados
//const float distNormal = 0.995; //5 grados
//const float distNormal = 0.9961947; //5 grados
//const int distNormal = 0.98480775; //10 grados
//const int distNormal = 0.96592583; //15 grados
//const int distNormal = 0.93969262; //20 grados
const float distRange = 0.2;
const float alfa = 0.0028;
const float beta = 30;
const float sumaBeta = 3;
const float delta = 1.25;
const float sumaDelta = 0.01;
const float delta2 = 2;
const float sumaDelta2 = 0.005;
const float umbralConvexidad = 0.20;
const float umbralCosenoNormales = 0.990;
const float umbralDistanciaComb = 0.4;
const float umbralCosenoComb = 0.98480775; //10 grados
const float deltaDepth = 4;
const float sumaDeltaDepth = 0.10;
const float umbralSueloAltura = 1;
const float umbralSueloNormal = 0.9;

const float pi = (float) 3.141592653589793238462643383279502884L;
const int coloresMaximos = 6;
const uint16_t blanco =     0x1111111111111111;
const uint16_t rojo =       0x1111100000000001;
const uint16_t verde =      0x0000011111000001;
const uint16_t azul =       0x0000000000111111;
const uint16_t amarillo =   0x1101111011000001;
const uint16_t morado =     0x1010000000101001;
const uint16_t cyan =       0x0000011001110011;
const uint16_t coloresBasicos[6] = {rojo, verde, azul, amarillo, morado, cyan};

void mapear(vector<Punto3D> &puntos, const TangoPointCloud *nube, TangoCameraIntrinsics intrinsics, int w, int h);

int procesar(vector<Punto3D> &puntos, map<int, Plano3D> &planos, map<int, Elemento3D> &elementos, bool(*f)(Punto3D *, Punto3D *), bool(*f2)(Punto3D *, Punto3D *), double timestampDepth, int w, int h, set<int> &suelo, map<int, int> &relevantes, int modoVista);

void calcularNormales(vector<Punto3D> &puntos, int w, int h, vector<float> integralX, vector<float> integralY, vector<float> integralZ, vector<int> vecinos);

int calcularMapaDinamicoDeVecinos(vector<Punto3D> &puntos, vector<int> &vecinos, int w, int h);

void calcularIntegralX(vector<Punto3D> &puntos, vector<float> &integral, int w, int h);

void calcularIntegralY(vector<Punto3D> &puntos, vector<float> &integral, int w, int h);

void calcularIntegralZ(vector<Punto3D> &puntos, vector<float> &integral, int w, int h);

float calcularSEnLaIntegral(vector<float> &integral, int w, int h, int m, int n, int r);

int aplicarConnectedComponentsLabeling(vector<Punto3D> &puntos, bool(*f)(Punto3D *, Punto3D *), vector<int> &padres, int w, int h);

int aplicarConnectedComponentsLabelingDepth(vector<Punto3D> &puntos, bool(*f)(Punto3D *, Punto3D *), vector<int> &padres, vector<int> &etiquetas, int w, int h);

void unionEtiquetas(int et1, int et2, vector<int> &padres);

int reducirPadres(vector<Punto3D> &puntos, vector<int> &padres, vector<int> &contadorPuntos);

int reducirPadresYAplicarEtiquetasDepth(vector<Punto3D> &puntos, vector<int> &padres, vector<int> &etiquetas, vector<int> &contadorPuntos);

int asignarPuntosAPlanos(vector<Punto3D> &puntos, map<int, Plano3D> &planos, vector<int> &contadorPuntos);

int asignarPuntosAElementos(vector<Punto3D> &puntos, map<int, Elemento3D> &elementos, vector<int> &contadorPuntos);

int borrarPlanosNoValidos(map<int, Plano3D> &planos);

void extenderPlanos(vector<Punto3D> &puntos, int w, int h);

void recalcularParametrosDelPlano(map<int, Plano3D> &planos);

int combinarPlanos(map<int, Plano3D> &planos);

void detectarSuelo(map<int, Plano3D> &planos, set<int> &suelo, double timestampDepth);

void detectarRelevantes(vector<Punto3D> puntos, map<int, Plano3D> &planos, map<int, Elemento3D> &elementos, set<int> &suelo, map<int, int> &relevantes);

float puntuarElemento(Elemento3D *elemento, int escalaTamaño, float maxX, float maxY, float maxZ);

bool comparadorProfundidad(Punto3D *punto1, Punto3D *punto2);

bool comparadorNormales(Punto3D *punto1, Punto3D *punto2);

void convolucionProfundidadGaussiana3(vector<Punto3D> &puntos, int w, int h);

void convolucionProfundidadGaussiana5(vector<Punto3D> &puntos, int w, int h);

void dibujarNumero(vector<uint16_t> &imagen, int w, int h, unsigned num[], int wM, int hM, int x, int y, int tam, int color);

void imprimirNumero(vector<uint16_t> &imagen, int n, int w, int h, int x, int y);

void colorearPorValidos(vector<Punto3D> &puntos, vector<uint16_t> &imagen, int w, int h, int escala);

void colorearPorCoordenadas(vector<Punto3D> &puntos, vector<uint16_t> &imagen, int w, int h, int escala);

void colorearPorNormales(vector<Punto3D> &puntos, vector<uint16_t> &imagen, int w, int h, int escala);

void colorearPorNormalesX(vector<Punto3D> &puntos, vector<uint16_t> &imagen, int w, int h, int escala);

void colorearPorNormalesY(vector<Punto3D> &puntos, vector<uint16_t> &imagen, int w, int h, int escala);

void colorearPorNormalesZ(vector<Punto3D> &puntos, vector<uint16_t> &imagen, int w, int h, int escala);

void colorearPorProfundidad(vector<Punto3D> &puntos, vector<uint16_t> &imagen);

void colorearPorEtiqueta(vector<Punto3D> &puntos, vector<uint16_t> &imagen, int w, int h, int escala);

void colorearPorEtiquetaRelevantes(vector<Punto3D> &puntos, vector<uint16_t> &imagen, int w, int h, int escala, set<int> &suelo, map<int, int> &relevantes);

void colorearPorVecinos(vector<int> &vecinos, vector<uint16_t> &imagen);

float getMaxDepth(vector<Punto3D> puntos);

float getMaxZ(vector<Punto3D> puntos);

float getMaxY(vector<Punto3D> puntos);

float getMaxX(vector<Punto3D> puntos);

float getMinZ(vector<Punto3D> puntos);

float getMinY(vector<Punto3D> puntos);

float getMinX(vector<Punto3D> puntos);


#endif //PRUEBA_CPP_SEGMENTACION_H
