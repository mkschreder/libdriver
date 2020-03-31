/** :ms-top-comment
 *  _____ _       _             ____
 * |  ___| |_   _(_)_ __   __ _| __ )  ___ _ __ __ _ _ __ ___   __ _ _ __
 * | |_  | | | | | | '_ \ / _` |  _ \ / _ \ '__/ _` | '_ ` _ \ / _` | '_ \
 * |  _| | | |_| | | | | | (_| | |_) |  __/ | | (_| | | | | | | (_| | | | |
 * |_|   |_|\__, |_|_| |_|\__, |____/ \___|_|  \__, |_| |_| |_|\__,_|_| |_|
 *          |___/         |___/                |___/
 **/

extern "C" {

}

#include "mock/timestamp.cpp"
#include "mock/regmap.cpp"
#include "mock/thread.cpp"
#include "mock/gpio.cpp"

class MPU6050Test : public ::testing::Test {
	protected:
	void SetUp() {
		//fb_init(&fb);
	}
	void TearDown() {
	}
	void clock(unsigned cycles) {

	}
};

TEST_F(MPU6050Test, check_init) {
}

int main(int argc, char **argv) {
	::testing::InitGoogleMock(&argc, argv);
	return RUN_ALL_TESTS();
}
