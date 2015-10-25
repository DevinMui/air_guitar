#include <SFML/Audio.hpp>
#include <SFML/System.hpp>
#include <iostream>
#define SFML_CLOCK_HPP
#define SFML_SOUNDBUFFER_HPP
using namespace std;

int main() {
    sf::Clock clock;
    sf::Time elapsed = clock.getElapsedTime();

    sf::SoundBuffer buffer;
    buffer.loadFromFile("1A.wav"); //note file name
    sf::Sound sound;
    sound.setBuffer(buffer);

    while (elapsed.asSeconds() < 0.6) {
        sound.play();
        elapsed = clock.getElapsedTime();
    }

    return 0;
}
