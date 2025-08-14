
// struct ControllerInput {
//   int L_x, L_y, R_x, R_y;
// };
// ControllerInput controller_input = { 0, 0, 0, 0 };

// void checkSerial1Input() {

//   if (Serial.available() > 0) {
//     // 改行文字まで文字列として一括で読み込む
//     String input_string = Serial.readStringUntil('\n');

//     input_string.trim();

//     // sscanfを使って、文字列から4つのint値を安全に解析(パース)する
//     int parsed_count = sscanf(input_string.c_str(), "%d,%d,%d,%d",
//                               &controller_input.L_x,
//                               &controller_input.L_y,
//                               &controller_input.R_x,
//                               &controller_input.R_y);

//     // NOTE: 4つの値が正しく読み取れた場合のみ、値を採用する。
//     // これにより、不完全なデータによる誤動作を防ぐ。
//     if (parsed_count != 4) {
//       // 読み取りに失敗した場合、値をリセットする
//       controller_input.L_x = 0.0f;
//       controller_input.L_y = 0.0f;
//       controller_input.R_x = 0.0f;
//       controller_input.R_y = 0.0f;
//     }
//   }
// }
