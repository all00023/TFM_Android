#pragma once

using namespace std;

class Plano3D;
class Elemento3D;

class Punto3D{

private:
    bool valido;
    bool normalValida;
    float x;
    float y;
    float z;
    float depth;
    float nx;
    float ny;
    float nz;
    float nd;
    int etiqueta;
    Plano3D* plano;
    Elemento3D* elemento;
    int auxiliarPuntosValidos;
    int posicionOriginal;

public:
    Punto3D(){
        plano = NULL;
        elemento = NULL;
        x = 0;
        y = 0;
        z = 0;
        depth = 0;
        valido = false;
        normalValida = false;
        etiqueta = 0;
        auxiliarPuntosValidos = 0;
        posicionOriginal = 9999999999;
    }

    ~Punto3D(){}

    void calcularPunto(){
        if (auxiliarPuntosValidos == 1){
            valido = true;
            calcularDepth();
        } else if (auxiliarPuntosValidos > 1){
            x = x / auxiliarPuntosValidos;
            y = y / auxiliarPuntosValidos;
            z = z / auxiliarPuntosValidos;
            valido = true;
            calcularDepth();
        }
    }

    int getPuntosValidos(){
        return auxiliarPuntosValidos;
    }

    void addPuntoValido(float x, float y, float z){
        this->x += x;
        this->y += y;
        this->z += z;
        auxiliarPuntosValidos++;
    }

    void setPuntoPosicion(float x, float y, float z, int pos){
        this->x = x;
        this->y = y;
        this->z = z;
        posicionOriginal = pos;
        valido = true;
    }

    int getPosicionOriginal(){
        return posicionOriginal;
    }

    void setPosicionOriginal(int pos){
        posicionOriginal = pos;
    }

    bool getValido(){
        return valido;
    }
    bool getNormalValida(){
        return normalValida;
    }
    float getX(){
        return x;
    }

    float getY(){
        return y;
    }

    float getZ(){
        return z;
    }

    float getDepth(){
        return depth;
    }

    float getNX(){
        return nx;
    }

    float getNY(){
        return ny;
    }

    float getNZ(){
        return nz;

    }float getND(){
        return nd;
    }

    int getEtiqueta(){
        return etiqueta;
    }

    Plano3D* getPlano(){
        return plano;
    }

    Elemento3D* getElemento(){
        return elemento;
    }

    void setValido(bool valido){
        this->valido = valido;
    }

    void setNormalValida(bool normalValida){
        this->normalValida = normalValida;
    }

    void setX(float x){
        this->x = x;
    }

    void setY(float y){
        this->y = y;
    }

    void setZ(float z){
        this->z = z;
    }

    void setDepth(float depth){
        this->depth = depth;
    }

    void setNX(float nx){
        this->nx = nx;
    }

    void setNY(float ny){
        this->ny = ny;
    }

    void setNZ(float nz){
        this->nz = nz;
    }

    void setND(float nd){
        this->nd = nd;
    }

    void setEtiqueta(int etiqueta){
        this->etiqueta = etiqueta;
    }

    void setPlano(Plano3D* plano){
        this->plano = plano;
    }

    void setElemento(Elemento3D* elemento){
        this->elemento = elemento;
    }

    void calcularND(){
        nd = -(x * nx) - (y * ny) - (z * nz);
    }

    void normalizarVectorNormal(){
        float modulo = moduloVectorNormal();
        nx /= modulo;
        ny /= modulo;
        nz /= modulo;

    }

    float moduloVectorNormal(){
        return sqrt(nx * nx + ny * ny + nz * nz);
    }

    void calcularDepth(){
        depth = sqrt(x * x + y * y + z * z);
    }

    float getDotProduct(Punto3D *punto2){
        return nx*punto2->nx + ny * punto2->ny + nz * punto2->nz;
    }

    float getNDMenosND(Punto3D *punto2){
        return abs(nd - punto2->nd);
    }

};

