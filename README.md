# 🚀 sensor_csr

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.17593098.svg)](https://doi.org/10.5281/zenodo.17593098)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Arduino](https://img.shields.io/badge/Arduino-Compatible-blue.svg)](https://www.arduino.cc/)
[![C++](https://img.shields.io/badge/Language-C++-brightgreen.svg)](https://isocpp.org/)

Biblioteca para sensores **QTR genéricos (não-Pololu)** desenvolvida por  
**César Augusto Victor** — Mestrando em Engenharia Elétrica e de Computação (UFC Sobral)  
📧 cesartri2012@gmail.com  

📘 **DOI:** [10.5281/zenodo.17593098](https://doi.org/10.5281/zenodo.17593098)

---

## 🧩 Sobre a Biblioteca

A **sensor_csr** é uma biblioteca para **Arduino/C++** desenvolvida para uso com barras de sensores **QTR genéricos**, amplamente empregadas em **robôs seguidores de linha**, como mostrado abaixo:

> *Demonstração do sensor_csr (imagem em `/docs/sensor.jpg`)*

Ela oferece uma interface simples e compatível com a biblioteca **QTRSensors da Pololu**, porém otimizada para sensores **analógicos de baixo custo**, com foco em fácil integração com projetos de robótica.

---

## ⚙️ Principais Recursos

✅ Calibração automática individual de cada sensor  
✅ Leitura analógica normalizada entre **0–1000**  
✅ Conversão binária automática (`0 = branco` / `1 = preto`)  
✅ Cálculo de erro discreto no intervalo **-4000 a +4000**  
✅ Função direta `ErroSensor()` para integração simples  
✅ Compatível com **DRV8833** e **L298N**  
✅ Suporte completo a **pistas tracejadas (GAP detection)**  

---

## 🤖 Códigos de Exemplo Incluídos

| Código | Driver  | Descrição |
|:------:|:--------:|:----------|
| `robot2` | DRV8833 | Controle PD dinâmico com **GAP detection** baseado nos bits `00000000` |
| `robotL298N` | L298N | Versão adaptada para drivers **H-Bridge clássicos**, com a mesma lógica de controle |

---

## 💡 Sistema de GAP Detection

Durante a leitura, se todos os sensores retornam `00000000`, o robô interpreta como um **espaço tracejado (gap)** e:

1. Avança suavemente por um ciclo;  
2. Caso o gap continue, o controle usa o **último erro válido (`lastNonZeroErro`)** para corrigir a trajetória automaticamente.

🧭 Isso garante **passagem fluida** por tracejados sem perder a linha nem gerar oscilações.

---

## 🔍 Como Funciona

Cada sensor lê um valor analógico **(0–1023)** proporcional à luz refletida:

| Cor da Superfície | Intensidade | Valor Analógico | Bit |
|:------------------|:-------------|:----------------|:----|
| Branco | Alta reflexão | Alto | 0 |
| Preto | Baixa reflexão | Baixo | 1 |

Durante a calibração (`qtr.calibrate()`), a biblioteca coleta valores mínimos e máximos e normaliza tudo entre **0 e 1000**.

Os limiares de cor devem ser definidos no seu código `.ino`:

```cpp
#define LIMIAR_BRANCO 600
#define LIMIAR_PRETO  900







