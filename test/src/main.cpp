
extern void test_one_wire_lib_setup();
extern void test_one_wire_lib_loop();
extern void test_dallas_temp_lib_setup();
extern void test_dallas_temp_lib_loop();




void setup(void) {
    test_dallas_temp_lib_setup();
}

void loop() {
    test_dallas_temp_lib_loop();
}
