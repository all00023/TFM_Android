#pragma once
#include "Punto3D.h"

using namespace std;

class Plano3D: public Elemento3D{

private:
	bool valido;

	double sumaNX;
	double sumaNY;
	double sumaNZ;

	float nx;
	float ny;
	float nz;
	float d;

	float depthCentro;
	set<Plano3D*> planosColindantes;

	float divisorDistanciaPunto;

public:
	Plano3D(int etiqueta) : Elemento3D(etiqueta){
		valido = false;
		sumaNX = 0;
		sumaNY = 0;
		sumaNZ = 0;
	}

	float getEtiqueta(){
		return etiqueta;
	}

	float getNX(){
		return nx;
	}

	float getNY(){
		return ny;
	}

	float getNZ(){
		return nz;

	}

	float getD(){
		return d;
	}

	float getDepthCentro(){
		return depthCentro;
	}

	set<Plano3D*> getPlanosColindantes(){
		return planosColindantes;
	}

	void addPunto(Punto3D *p){

		sumaNX += p->getNX();
		sumaNY += p->getNY();
		sumaNZ += p->getNZ();

		sumaX += p->getX();
		sumaY += p->getY();
		sumaZ += p->getZ();

		puntosInternos.insert(p);
		p->setEtiqueta(etiqueta);

	}

	void addPlanoColindante(Plano3D *p){

		planosColindantes.insert(p);

	}

	void calcularParametrosDelPlano(){

		calcularParametrosDelElemento();

		nx = sumaNX / puntosInternos.size();
		ny = sumaNY / puntosInternos.size();
		nz = sumaNZ / puntosInternos.size();

		normalizarVectorNormal();

		d = -(cx * nx) - (cy * ny) - (cz * nz);

		divisorDistanciaPunto = sqrt(nx * nx + ny * ny + nz * nz);

		depthCentro = sqrt(cx * cx + cy * cy + cz * cz);

	}

	float moduloVectorNormal(){
		return sqrt(nx * nx + ny * ny + nz * nz);
	}

	void normalizarVectorNormal(){
		float modulo = moduloVectorNormal();
		nx /= modulo;
		ny /= modulo;
		nz /= modulo;

	}

	bool calcularSiValido(float umbralConvexidad, int minPuntos, float umbralCosenoNormales){

		valido = true;

		if (puntosInternos.size() < minPuntos){

			valido = false;

		} else{

			int noValidos = 0;
			int limite = puntosInternos.size() * umbralConvexidad;
			set<Punto3D*>::iterator it = puntosInternos.begin();

			while (it != puntosInternos.end() && noValidos < limite){

				if (getDotProductPunto(*it) < umbralCosenoNormales)
					noValidos++;

				it++;
			}

			if (noValidos >= limite){
				valido = false;
			}

		}

		return valido;
	}

	void combinarPlanos(Plano3D* plano2){

		sumaNX += plano2->sumaNX;
		sumaNY += plano2->sumaNY;
		sumaNZ += plano2->sumaNZ;

		sumaX += plano2->sumaX;
		sumaY += plano2->sumaY;
		sumaZ += plano2->sumaZ;

		puntosInternos.insert(plano2->puntosInternos.begin(), plano2->puntosInternos.end());
		planosColindantes.insert(plano2->planosColindantes.begin(), plano2->planosColindantes.end());

		for (set<Punto3D*>::iterator it = plano2->puntosInternos.begin(); it != plano2->puntosInternos.end(); ++it){
			(*it)->setPlano(const_cast<Plano3D*>(this));
			(*it)->setEtiqueta(etiqueta);
		}

		plano2->puntosInternos.clear();
		plano2->planosColindantes.clear();

		//for (set<Plano3D*>::iterator it = plano2->planosColindantes.begin(); it != plano2->planosColindantes.end(); ++it){
		//	(*it)->planosColindantes.erase(plano2);
		//	(*it)->planosColindantes.insert(const_cast<Plano3D*>(this));
		//}

	}

	float getDotProductPunto(Punto3D *punto){
		return abs(nx*punto->getNX() + ny * punto->getNY() + nz * punto->getNZ());
	}

	float getDotProductPlano(Plano3D *plano){
		return abs(nx*plano->getNX() + ny * plano->getNY() + nz * plano->getNZ());
	}

	float getDistanciaAPunto(Punto3D *punto){
		return abs(nx * punto->getX() + ny * punto->getY() + nz * punto->getZ() + d) / divisorDistanciaPunto;
	}

	float getDistanciaAPuntoCentralDelPlano(Plano3D *plano){
		return abs(nx * plano->cx + ny * plano->cy + nz * plano->cz + d) / divisorDistanciaPunto;
	}

	float getNDMenosND(Plano3D *plano2){
		return abs(d - plano2->d);
	}

	float isColindante(Plano3D *plano2){
		set<Plano3D*>::iterator col = planosColindantes.find(plano2);
		return col != planosColindantes.end();
	}

};