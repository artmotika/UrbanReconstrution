#ifndef URBAN_RECONSTRUCTION_TEXTUREMAPPINGCONSTS_H
#define URBAN_RECONSTRUCTION_TEXTUREMAPPINGCONSTS_H
/*
Эти константы относятся к параметрам материала в трехмерной графике,
используемые в формате файлов OBJ. Вот их общее значение:

- KA_R, KA_G, KA_B: Коэффициенты амбиентного отражения материала (ambient reflectivity) в красной, зеленой и синей компонентах соответственно.
- KD_R, KD_G, KD_B: Коэффициенты диффузного отражения материала (diffuse reflectivity) в красной, зеленой и синей компонентах соответственно.
- KS_R, KS_G, KS_B: Коэффициенты зеркального отражения материала (specular reflectivity) в красной, зеленой и синей компонентах соответственно.
- D: Прозрачность материала (dissolve factor).
- NS: Коэффициент блеска материала (specular exponent).
- ILLUM: Номер модели освещения, которая используется для материала.
*/
const float KA_R = 0.2f;
const float KA_G = 0.2f;
const float KA_B = 0.2f;

const float KD_R = 0.8f;
const float KD_G = 0.8f;
const float KD_B = 0.8f;

const float KS_R = 1.0f;
const float KS_G = 1.0f;
const float KS_B = 1.0f;

const float D = 1.0f;
const float NS = 75.0f;
const int ILLUM = 2;
#endif //URBAN_RECONSTRUCTION_TEXTUREMAPPINGCONSTS_H
