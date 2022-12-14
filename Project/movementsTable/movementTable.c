//
// Created by 35802 on 2022/8/31.
//
#include "Servo.h"
#include "movementTable.h"
void movement_highlifting_forward(){
    moveServos(18, 12, 1, 480, 2, 634, 3, 340, 4, 488, 5, 495, 6, 560, 10, 500, 11, 632, 12, 279, 13, 436, 14, 513, 15, 283, 7, 494, 8, 687, 9, 733, 16, 496, 17, 510, 18, 781);
    HAL_Delay(12);
    moveServos(18, 12, 1, 472, 2, 586, 3, 278, 4, 503, 5, 495, 6, 559, 10, 491, 11, 589, 12, 252, 13, 428, 14, 507, 15, 267, 7, 509, 8, 642, 9, 692, 16, 487, 17, 515, 18, 800);
    HAL_Delay(12);
    moveServos(18, 12, 1, 465, 2, 505, 3, 169, 4, 519, 5, 495, 6, 558, 10, 482, 11, 515, 12, 189, 13, 421, 14, 500, 15, 247, 7, 525, 8, 566, 9, 614, 16, 478, 17, 518, 18, 817);
    HAL_Delay(12);
    moveServos(18, 12, 1, 472, 2, 512, 3, 191, 4, 503, 5, 570, 6, 640, 10, 491, 11, 512, 12, 176, 13, 428, 14, 585, 15, 345, 7, 509, 8, 566, 9, 616, 16, 487, 17, 592, 18, 898);
    HAL_Delay(12);
    moveServos(18, 12, 1, 480, 2, 518, 3, 209, 4, 488, 5, 614, 6, 683, 10, 500, 11, 508, 12, 162, 13, 436, 14, 636, 15, 400, 7, 494, 8, 567, 9, 616, 16, 496, 17, 635, 18, 933);
    HAL_Delay(12);
    moveServos(18, 12, 1, 488, 2, 522, 3, 225, 4, 472, 5, 570, 6, 640, 10, 507, 11, 502, 12, 145, 13, 444, 14, 594, 15, 373, 7, 478, 8, 566, 9, 616, 16, 503, 17, 584, 18, 861);
    HAL_Delay(12);
    moveServos(18, 12, 1, 497, 2, 524, 3, 240, 4, 456, 5, 495, 6, 558, 10, 514, 11, 494, 12, 126, 13, 453, 14, 520, 15, 311, 7, 462, 8, 566, 9, 614, 16, 510, 17, 497, 18, 734);
    HAL_Delay(12);
    moveServos(18, 12, 1, 488, 2, 594, 3, 309, 4, 472, 5, 495, 6, 559, 10, 507, 11, 580, 12, 223, 13, 444, 14, 518, 15, 298, 7, 478, 8, 642, 9, 692, 16, 503, 17, 505, 18, 759);
    HAL_Delay(12);
}
void movement_normal_forward(){
    moveServos(18, 12, 1, 480, 2, 577, 3, 279, 4, 488, 5, 495, 6, 560, 10, 500, 11, 571, 12, 225, 13, 436, 14, 513, 15, 283, 7, 494, 8, 628, 9, 679, 16, 496, 17, 510, 18, 781);
    HAL_Delay(12);
    moveServos(18, 12, 1, 472, 2, 565, 3, 254, 4, 503, 5, 495, 6, 559, 10, 491, 11, 567, 12, 230, 13, 428, 14, 507, 15, 267, 7, 509, 8, 620, 9, 671, 16, 487, 17, 515, 18, 800);
    HAL_Delay(12);
    moveServos(18, 12, 1, 465, 2, 505, 3, 169, 4, 519, 5, 495, 6, 558, 10, 482, 11, 515, 12, 189, 13, 421, 14, 500, 15, 247, 7, 525, 8, 566, 9, 614, 16, 478, 17, 518, 18, 817);
    HAL_Delay(12);
    moveServos(18, 12, 1, 472, 2, 512, 3, 191, 4, 503, 5, 548, 6, 617, 10, 491, 11, 512, 12, 176, 13, 428, 14, 563, 15, 323, 7, 509, 8, 566, 9, 616, 16, 487, 17, 569, 18, 870);
    HAL_Delay(12);
    moveServos(18, 12, 1, 480, 2, 518, 3, 209, 4, 488, 5, 556, 6, 626, 10, 500, 11, 508, 12, 162, 13, 436, 14, 576, 15, 346, 7, 494, 8, 567, 9, 616, 16, 496, 17, 574, 18, 863);
    HAL_Delay(12);
    moveServos(18, 12, 1, 488, 2, 522, 3, 225, 4, 472, 5, 548, 6, 617, 10, 507, 11, 502, 12, 145, 13, 444, 14, 572, 15, 352, 7, 478, 8, 566, 9, 616, 16, 503, 17, 561, 18, 833);
    HAL_Delay(12);
    moveServos(18, 12, 1, 497, 2, 524, 3, 240, 4, 456, 5, 495, 6, 558, 10, 514, 11, 494, 12, 126, 13, 453, 14, 520, 15, 311, 7, 462, 8, 566, 9, 614, 16, 510, 17, 497, 18, 734);
    HAL_Delay(12);
    moveServos(18, 12, 1, 488, 2, 573, 3, 286, 4, 472, 5, 495, 6, 559, 10, 507, 11, 558, 12, 202, 13, 444, 14, 518, 15, 298, 7, 478, 8, 620, 9, 671, 16, 503, 17, 505, 18, 759);
    HAL_Delay(12);
}
void movement_rotate_left(){
    moveServos(18, 2, 1, 502, 2, 553, 3, 239, 4, 488, 5, 564, 6, 613, 10, 477, 11, 546, 12, 189, 13, 408, 14, 477, 15, 254, 7, 494, 8, 489, 9, 551, 16, 523, 17, 474, 18, 743);
    HAL_Delay(2);
    moveServos(18, 2, 1, 505, 2, 543, 3, 231, 4, 492, 5, 563, 6, 612, 10, 480, 11, 556, 12, 196, 13, 407, 14, 489, 15, 264, 7, 487, 8, 491, 9, 552, 16, 520, 17, 462, 18, 730);
    HAL_Delay(2);
    moveServos(18, 2, 1, 507, 2, 532, 3, 221, 4, 496, 5, 560, 6, 610, 10, 483, 11, 564, 12, 202, 13, 407, 14, 502, 15, 274, 7, 482, 8, 495, 9, 556, 16, 517, 17, 451, 18, 719);
    HAL_Delay(2);
    moveServos(18, 2, 1, 508, 2, 520, 3, 210, 4, 500, 5, 556, 6, 607, 10, 487, 11, 571, 12, 206, 13, 407, 14, 515, 15, 284, 7, 477, 8, 501, 9, 561, 16, 512, 17, 443, 18, 709);
    HAL_Delay(2);
    moveServos(18, 2, 1, 508, 2, 508, 3, 199, 4, 504, 5, 549, 6, 602, 10, 491, 11, 576, 12, 210, 13, 408, 14, 528, 15, 294, 7, 472, 8, 509, 9, 569, 16, 507, 17, 436, 18, 702);
    HAL_Delay(2);
    moveServos(18, 2, 1, 508, 2, 495, 3, 188, 4, 507, 5, 541, 6, 596, 10, 495, 11, 579, 12, 212, 13, 410, 14, 540, 15, 302, 7, 469, 8, 519, 9, 577, 16, 502, 17, 432, 18, 697);
    HAL_Delay(2);
    moveServos(18, 2, 1, 507, 2, 484, 3, 176, 4, 510, 5, 532, 6, 589, 10, 500, 11, 580, 12, 212, 13, 413, 14, 551, 15, 310, 7, 466, 8, 531, 9, 587, 16, 496, 17, 431, 18, 696);
    HAL_Delay(2);
    moveServos(18, 2, 1, 504, 2, 473, 3, 166, 4, 513, 5, 521, 6, 580, 10, 504, 11, 579, 12, 212, 13, 416, 14, 561, 15, 317, 7, 465, 8, 543, 9, 597, 16, 489, 17, 432, 18, 697);
    HAL_Delay(2);
    moveServos(18, 2, 1, 501, 2, 463, 3, 156, 4, 515, 5, 510, 6, 571, 10, 508, 11, 576, 12, 210, 13, 419, 14, 569, 15, 323, 7, 465, 8, 556, 9, 607, 16, 484, 17, 436, 18, 702);
    HAL_Delay(2);
    moveServos(18, 2, 1, 496, 2, 455, 3, 148, 4, 516, 5, 497, 6, 561, 10, 512, 11, 571, 12, 206, 13, 423, 14, 576, 15, 328, 7, 465, 8, 568, 9, 617, 16, 479, 17, 443, 18, 709);
    HAL_Delay(2);
    moveServos(18, 2, 1, 491, 2, 449, 3, 142, 4, 516, 5, 485, 6, 550, 10, 516, 11, 564, 12, 202, 13, 427, 14, 581, 15, 331, 7, 466, 8, 581, 9, 627, 16, 474, 17, 451, 18, 719);
    HAL_Delay(2);
    moveServos(18, 2, 1, 486, 2, 445, 3, 138, 4, 516, 5, 472, 6, 540, 10, 519, 11, 556, 12, 196, 13, 431, 14, 584, 15, 333, 7, 468, 8, 593, 9, 635, 16, 471, 17, 462, 18, 730);
    HAL_Delay(2);
    moveServos(18, 2, 1, 480, 2, 444, 3, 136, 4, 515, 5, 460, 6, 529, 10, 522, 11, 546, 12, 189, 13, 436, 14, 585, 15, 334, 7, 471, 8, 603, 9, 643, 16, 468, 17, 474, 18, 743);
    HAL_Delay(2);
    moveServos(18, 2, 1, 473, 2, 445, 3, 138, 4, 512, 5, 449, 6, 519, 10, 525, 11, 535, 12, 181, 13, 440, 14, 584, 15, 333, 7, 474, 8, 613, 9, 650, 16, 467, 17, 486, 18, 756);
    HAL_Delay(2);
    moveServos(18, 2, 1, 468, 2, 449, 3, 142, 4, 509, 5, 439, 6, 510, 10, 527, 11, 523, 12, 172, 13, 444, 14, 581, 15, 331, 7, 477, 8, 621, 9, 656, 16, 467, 17, 499, 18, 769);
    HAL_Delay(2);
    moveServos(18, 2, 1, 463, 2, 455, 3, 148, 4, 504, 5, 431, 6, 502, 10, 528, 11, 510, 12, 162, 13, 448, 14, 576, 15, 328, 7, 481, 8, 628, 9, 661, 16, 467, 17, 512, 18, 782);
    HAL_Delay(2);
    moveServos(18, 2, 1, 458, 2, 463, 3, 156, 4, 499, 5, 425, 6, 497, 10, 528, 11, 497, 12, 152, 13, 452, 14, 569, 15, 323, 7, 485, 8, 632, 9, 664, 16, 468, 17, 525, 18, 794);
    HAL_Delay(2);
    moveServos(18, 2, 1, 455, 2, 473, 3, 166, 4, 494, 5, 421, 6, 493, 10, 528, 11, 484, 12, 142, 13, 455, 14, 561, 15, 317, 7, 489, 8, 635, 9, 666, 16, 470, 17, 537, 18, 806);
    HAL_Delay(2);
    moveServos(18, 2, 1, 452, 2, 484, 3, 176, 4, 488, 5, 419, 6, 492, 10, 527, 11, 471, 12, 132, 13, 458, 14, 551, 15, 310, 7, 494, 8, 636, 9, 667, 16, 473, 17, 549, 18, 816);
    HAL_Delay(2);
    moveServos(18, 2, 1, 451, 2, 495, 3, 188, 4, 481, 5, 421, 6, 493, 10, 524, 11, 460, 12, 122, 13, 461, 14, 540, 15, 302, 7, 498, 8, 635, 9, 666, 16, 476, 17, 558, 18, 825);
    HAL_Delay(2);
    moveServos(18, 2, 1, 451, 2, 508, 3, 199, 4, 476, 5, 425, 6, 497, 10, 521, 11, 449, 12, 114, 13, 463, 14, 528, 15, 294, 7, 502, 8, 632, 9, 664, 16, 479, 17, 567, 18, 833);
    HAL_Delay(2);
    moveServos(18, 2, 1, 451, 2, 520, 3, 210, 4, 471, 5, 431, 6, 502, 10, 516, 11, 440, 12, 106, 13, 464, 14, 515, 15, 284, 7, 506, 8, 628, 9, 661, 16, 483, 17, 574, 18, 839);
    HAL_Delay(2);
    moveServos(18, 2, 1, 452, 2, 532, 3, 221, 4, 466, 5, 439, 6, 510, 10, 511, 11, 434, 12, 101, 13, 464, 14, 502, 15, 274, 7, 510, 8, 621, 9, 656, 16, 487, 17, 579, 18, 843);
    HAL_Delay(2);
    moveServos(18, 2, 1, 454, 2, 543, 3, 231, 4, 463, 5, 449, 6, 519, 10, 506, 11, 430, 12, 97, 13, 464, 14, 489, 15, 264, 7, 513, 8, 613, 9, 650, 16, 491, 17, 582, 18, 846);
    HAL_Delay(2);
    moveServos(18, 2, 1, 457, 2, 553, 3, 239, 4, 460, 5, 460, 6, 529, 10, 500, 11, 428, 12, 96, 13, 463, 14, 477, 15, 254, 7, 516, 8, 603, 9, 643, 16, 496, 17, 583, 18, 847);
    HAL_Delay(2);
    moveServos(18, 2, 1, 460, 2, 562, 3, 247, 4, 459, 5, 472, 6, 540, 10, 493, 11, 430, 12, 97, 13, 460, 14, 465, 15, 244, 7, 519, 8, 593, 9, 635, 16, 500, 17, 582, 18, 846);
    HAL_Delay(2);
    moveServos(18, 2, 1, 463, 2, 570, 3, 254, 4, 459, 5, 485, 6, 550, 10, 488, 11, 434, 12, 101, 13, 457, 14, 455, 15, 236, 7, 521, 8, 581, 9, 627, 16, 504, 17, 579, 18, 843);
    HAL_Delay(2);
    moveServos(18, 2, 1, 467, 2, 577, 3, 259, 4, 459, 5, 497, 6, 561, 10, 483, 11, 440, 12, 106, 13, 452, 14, 446, 15, 228, 7, 522, 8, 568, 9, 617, 16, 508, 17, 574, 18, 839);
    HAL_Delay(2);
    moveServos(18, 2, 1, 471, 2, 581, 3, 263, 4, 460, 5, 510, 6, 571, 10, 478, 11, 449, 12, 114, 13, 447, 14, 440, 15, 223, 7, 522, 8, 556, 9, 607, 16, 512, 17, 567, 18, 833);
    HAL_Delay(2);
    moveServos(18, 2, 1, 475, 2, 584, 3, 265, 4, 462, 5, 521, 6, 580, 10, 475, 11, 460, 12, 122, 13, 442, 14, 436, 15, 219, 7, 522, 8, 543, 9, 597, 16, 515, 17, 558, 18, 825);
    HAL_Delay(2);
    moveServos(18, 2, 1, 480, 2, 585, 3, 266, 4, 465, 5, 532, 6, 589, 10, 472, 11, 471, 12, 132, 13, 436, 14, 435, 15, 218, 7, 521, 8, 531, 9, 587, 16, 518, 17, 549, 18, 816);
    HAL_Delay(2);
    moveServos(18, 2, 1, 484, 2, 584, 3, 265, 4, 468, 5, 541, 6, 596, 10, 471, 11, 484, 12, 142, 13, 429, 14, 436, 15, 219, 7, 518, 8, 519, 9, 577, 16, 521, 17, 537, 18, 806);
    HAL_Delay(2);
    moveServos(18, 2, 1, 488, 2, 581, 3, 263, 4, 471, 5, 549, 6, 602, 10, 471, 11, 497, 12, 152, 13, 424, 14, 440, 15, 223, 7, 515, 8, 509, 9, 569, 16, 523, 17, 525, 18, 794);
    HAL_Delay(2);
    moveServos(18, 2, 1, 492, 2, 577, 3, 259, 4, 475, 5, 556, 6, 607, 10, 471, 11, 510, 12, 162, 13, 419, 14, 446, 15, 228, 7, 510, 8, 501, 9, 561, 16, 524, 17, 512, 18, 782);
    HAL_Delay(2);
    moveServos(18, 2, 1, 496, 2, 570, 3, 254, 4, 479, 5, 560, 6, 610, 10, 472, 11, 523, 12, 172, 13, 414, 14, 455, 15, 236, 7, 505, 8, 495, 9, 556, 16, 524, 17, 499, 18, 769);
    HAL_Delay(2);
    moveServos(18, 2, 1, 499, 2, 562, 3, 247, 4, 483, 5, 563, 6, 612, 10, 474, 11, 535, 12, 181, 13, 411, 14, 465, 15, 244, 7, 500, 8, 491, 9, 552, 16, 524, 17, 486, 18, 756);
    HAL_Delay(2);
    moveServos(18, 2, 1, 502, 2, 553, 3, 239, 4, 488, 5, 564, 6, 613, 10, 477, 11, 546, 12, 189, 13, 408, 14, 477, 15, 254, 7, 494, 8, 489, 9, 551, 16, 523, 17, 474, 18, 743);
    HAL_Delay(2);
}
void movement_turn_left(){
    moveServos(18, 12, 1, 512, 2, 517, 3, 208, 4, 455, 5, 495, 6, 559, 10, 532, 11, 507, 12, 160, 13, 403, 14, 513, 15, 282, 7, 526, 8, 566, 9, 615, 16, 463, 17, 510, 18, 779);
    HAL_Delay(12);
    moveServos(18, 12, 1, 496, 2, 518, 3, 209, 4, 471, 5, 549, 6, 619, 10, 516, 11, 508, 12, 161, 13, 419, 14, 569, 15, 339, 7, 510, 8, 566, 9, 616, 16, 479, 17, 567, 18, 854);
    HAL_Delay(12);
    moveServos(18, 12, 1, 480, 2, 518, 3, 209, 4, 488, 5, 557, 6, 627, 10, 500, 11, 508, 12, 162, 13, 436, 14, 577, 15, 348, 7, 494, 8, 567, 9, 616, 16, 496, 17, 576, 18, 865);
    HAL_Delay(12);
    moveServos(18, 12, 1, 463, 2, 518, 3, 209, 4, 504, 5, 549, 6, 619, 10, 483, 11, 508, 12, 161, 13, 452, 14, 569, 15, 339, 7, 477, 8, 566, 9, 616, 16, 512, 17, 567, 18, 854);
    HAL_Delay(12);
    moveServos(18, 12, 1, 447, 2, 517, 3, 208, 4, 520, 5, 495, 6, 559, 10, 467, 11, 507, 12, 160, 13, 468, 14, 513, 15, 282, 7, 461, 8, 566, 9, 615, 16, 528, 17, 510, 18, 779);
    HAL_Delay(12);
    moveServos(18, 12, 1, 463, 2, 570, 3, 272, 4, 504, 5, 495, 6, 560, 10, 483, 11, 564, 12, 218, 13, 452, 14, 513, 15, 283, 7, 477, 8, 621, 9, 672, 16, 512, 17, 510, 18, 780);
    HAL_Delay(12);
    moveServos(18, 12, 1, 480, 2, 578, 3, 281, 4, 488, 5, 495, 6, 560, 10, 500, 11, 572, 12, 226, 13, 436, 14, 513, 15, 283, 7, 494, 8, 629, 9, 681, 16, 496, 17, 510, 18, 781);
    HAL_Delay(12);
    moveServos(18, 12, 1, 496, 2, 570, 3, 272, 4, 471, 5, 495, 6, 560, 10, 516, 11, 564, 12, 218, 13, 419, 14, 513, 15, 283, 7, 510, 8, 621, 9, 672, 16, 479, 17, 510, 18, 780);
    HAL_Delay(12);
}

