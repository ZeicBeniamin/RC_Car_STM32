/*
 * utils.c
 *
 *  Created on: 5 May 2021
 *      Author: Beniamin Zeic
 */

float map(float value, float l1, float h1, float l2, float h2) {
  return l2 + (value - l1) * (h2 - l2)/(h1 - l1);
}

float max(float val1, float val2) {
	return (val1 > val2 ? val1 : val2);
}
