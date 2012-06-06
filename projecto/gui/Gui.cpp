#include "Gui.h"

Gui::Gui(int argc, char* argv[]) {

	QApplication app(argc, argv);
	QWidget window;

	QTextEdit *textEdit = new QTextEdit;
	QPushButton *quitButton = new QPushButton("&Quit");
	QLabel * pointCloudLabel = new QLabel("Nuvem de Pontos");
	QLabel * eventsLabel = new QLabel("Nuvem de Pontos");

	QObject::connect(quitButton, SIGNAL(clicked()), qApp, SLOT(quit()));

	QVBoxLayout *layout = new QVBoxLayout;
	layout->addWidget(pointCloudLabel);
	layout->addWidget(textEdit);
	layout->addWidget(quitButton);

	window.setWindowTitle("Detector");

	window.setLayout(layout);
	window.show();

	// Executes the GUI
	app.exec();
}

void Gui::show() {

}

