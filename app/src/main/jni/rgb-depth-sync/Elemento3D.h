#pragma once
#include "Punto3D.h"

using namespace std;

class Elemento3D{

protected:
	int etiqueta;

	double sumaX;
	double sumaY;
	double sumaZ;

	float cx;
	float cy;
	float cz;

	set<Punto3D*> puntosInternos;

public:

	Elemento3D(int etiqueta){
		sumaX = 0;
		sumaY = 0;
		sumaZ = 0;
		this->etiqueta = etiqueta;
	}

	float getEtiqueta(){
		return etiqueta;
	}

	void liberarPuntos(){
		for (set<Punto3D*>::iterator it = puntosInternos.begin(); it != puntosInternos.end(); ++it){
			(*it)->setElemento(NULL);
			(*it)->setPlano(NULL);
			(*it)->setEtiqueta(0);
		}
	}


	void addPunto(Punto3D *p){

		sumaX += p->getX();
		sumaY += p->getY();
		sumaZ += p->getZ();

		puntosInternos.insert(p);
		p->setEtiqueta(etiqueta);

	}

	void calcularParametrosDelElemento(){

		cx = sumaX / puntosInternos.size();
		cy = sumaY / puntosInternos.size();
		cz = sumaZ / puntosInternos.size();

	}

	float getCX(){
		return cx;
	}

	float getCY(){
		return cy;
	}

	float getCZ(){
		return cz;
	}

	int getNumeroPuntos(){
		return puntosInternos.size();
	}

};