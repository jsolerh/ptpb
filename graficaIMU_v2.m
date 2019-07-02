%Función que toma datos por un puerto serie y dibuja las grafica por medio de la funcion plot

function [LinearX, LinearY, LinearZ, GyroscopeX, GyroscopeY, GyroscopeZ, EulerX, EulerY, EulerZ, n_muestras]=graficaIMU_v2 %la funcion recibe el # de muestras que debe tomar

close all; %Cierra todo lo que matlab tenga abierto
clc; %limpiar la pantalla
clear all;
% muestras=0;%Declara variable en la cual se van a guardar los valores
% n=n*9;
% LinearX=0;%Declara variable en la que se van a guardar los datos
% LinearY=0;%Declara variable en la que se van a guardar los datos
% LinearZ=0;%Declara variable en la que se van a guardar los datos
% GravityX=0;%Declara variable en la que se van a guardar los datos
% GravityY=0;%Declara variable en la que se van a guardar los datos
% GravityZ=0;%Declara variable en la que se van a guardar los datos
% EulerX=0;%Declara variable en la que se van a guardar los datos
% EulerY=0;%Declara variable en la que se van a guardar los datos
% EulerZ=0;%Declara variable en la que se van a guardar los datos

%Borra datos que se encuentren previos y vuelve a declarar el puerto y la
%velocidad de transmisión
delete(instrfind({'port'},{'COM3'})); %borrar cualquier puerto serial abierto
puerto=serial('COM3'); %declaro variable llamada puerto y se crea el com4
puerto.BaudRate=115200; %115200 %Establecer velocidad de transmisión
% puerto.ByteOrder = 'bigEndian';


fopen(puerto);%abre el puerto a utilizar
contador=1;

%configura la ventana donde se va a mostrar la grafica
figure('Name','IMU')%Nombre de la ventana
title('IMU'); %Titulo de la grafica
xlabel('Numero de Muestras'); %Leyenda o titulo del eje x
ylabel('Valor'); %Leyenda o titulo en el eje y
grid on; %Apagar cuadricula
hold on;
%Ciclo para capturando valores e ir realizando la grafica paso a paso

n_muestras = fscanf(puerto, '%32i');
%n_muestras = 9*n_muestras;
% fclose(puerto);
% fopen(puerto);


while contador<=n_muestras
    valor=fscanf(puerto,'%16i%16i%16i%16i%16i%16i%16i%16i%16i'); %Toma el valor recibido por el puerto y lo guarda en la variable
%     muestras(contador)=valorADC(1); %
    LinearX(contador)=valor(1)/100;
    LinearY(contador)=valor(2)/100;
    LinearZ(contador)=valor(3)/100;
    GyroscopeX(contador)=valor(4)/16;
    GyroscopeY(contador)=valor(5)/16;
    GyroscopeZ(contador)=valor(6)/16;
    EulerX(contador)=valor(7)/16;
    EulerY(contador)=valor(8)/16;
    EulerZ(contador)=valor(9)/16;
    contador=contador+1;
end


vectorM=(1:1:n_muestras);% vector que va a representar el numero de muestra en el eje X de la grafica
%plot(vectorM,LinearX,vectorM,LinearY,vectorM,LinearZ,vectorM,GravityX,vectorM,GravityY,vectorM,GravityZ,vectorM,EulerX,vectorM,EulerY,vectorM,EulerZ) %Grafica de las dos señales
%%
subplot(3,3,1)
plot(vectorM, LinearX);
title('Lienar X'); %Titulo de la grafica
xlabel('Muestra'); %Leyenda o titulo del eje x
ylabel('m/s^-2'); %Leyenda o titulo en el eje y
axis([0 n_muestras -10 10]);

subplot(3,3,2)
plot(vectorM, LinearY);
title('Lienar Y'); %Titulo de la grafica
xlabel('Muestra'); %Leyenda o titulo del eje x
ylabel('m/s^-2'); %Leyenda o titulo en el eje y
axis([0 n_muestras -10 10]);

subplot(3,3,3)
plot(vectorM, LinearZ);
title('Lienar Z'); %Titulo de la grafica
xlabel('Muestra'); %Leyenda o titulo del eje x
ylabel('m/s^-2'); %Leyenda o titulo en el eje y
axis([0 n_muestras -10 10]);

subplot(3,3,4)
plot(vectorM, GyroscopeX);
title('Gyroscope X'); %Titulo de la grafica
xlabel('Muestra'); %Leyenda o titulo del eje x
ylabel('Dps'); %Leyenda o titulo en el eje y
axis([0 n_muestras -1000 1000]);

subplot(3,3,5)
plot(vectorM, GyroscopeY);
title('Gyroscope Y'); %Titulo de la grafica
xlabel('Muestra'); %Leyenda o titulo del eje x
ylabel('Dps'); %Leyenda o titulo en el eje y
axis([0 n_muestras -1000 1000]);

subplot(3,3,6)
plot(vectorM, GyroscopeZ);
title('Gyroscope Z'); %Titulo de la grafica
xlabel('Muestra'); %Leyenda o titulo del eje x
ylabel('Dps'); %Leyenda o titulo en el eje y
axis([0 n_muestras -1000 1000]);

subplot(3,3,7)
plot(vectorM, EulerX);
title('Euler X'); %Titulo de la grafica
xlabel('Muestra'); %Leyenda o titulo del eje x
ylabel('Grados'); %Leyenda o titulo en el eje y
axis([0 n_muestras -360 360]);

subplot(3,3,8)
plot(vectorM, EulerY);
title('Euler Y'); %Titulo de la grafica
xlabel('Muestra'); %Leyenda o titulo del eje x
ylabel('Grados'); %Leyenda o titulo en el eje y
axis([0 n_muestras -360 360]);

subplot(3,3,9)
plot(vectorM, EulerZ);
title('Euler Z'); %Titulo de la grafica
xlabel('Muestra'); %Leyenda o titulo del eje x
ylabel('Grados'); %Leyenda o titulo en el eje y
axis([0 n_muestras -360 360]);

%cierra y borra el puerto utilizado, borra todas las variables utilizadas
fclose(puerto);
delete(puerto);
end