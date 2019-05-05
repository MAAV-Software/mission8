#!/bin/bash

SOFTWARE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && cd ../.. && pwd )"
QT5_DIR=$SOFTWARE_DIR/thirdparty/Qt5

QT_SCRIPT=/tmp/qt-noninteractive-install-linux.qs
mkdir -p $QT5_DIR

echo $QT5_DIR

# generate script for automatic Qt installation; customizes the install location via $QT_SDK_DIR
function write_qt_script() {
cat << EOF > $QT_SCRIPT
function Controller() {
    installer.autoRejectMessageBoxes();
    installer.installationFinished.connect(function() {
        gui.clickButton(buttons.NextButton);
    })
}

Controller.prototype.WelcomePageCallback = function() {
    // click delay here because the next button is initially disabled for ~1 second
    gui.clickButton(buttons.NextButton, 3000);
}

Controller.prototype.CredentialsPageCallback = function() {
   var widget = gui.currentPageWidget();
   widget.loginWidget.EmailLineEdit.setText("");
   widget.loginWidget.PasswordLineEdit.setText("");
   gui.clickButton(buttons.NextButton, 500);
}

Controller.prototype.IntroductionPageCallback = function() {
    gui.clickButton(buttons.NextButton);
}

Controller.prototype.TargetDirectoryPageCallback = function()
{    var widget = gui.currentPageWidget();
        if (widget != null) {
       widget.TargetDirectoryLineEdit.setText("$QT5_DIR");
   }
   gui.clickButton(buttons.NextButton);
}

Controller.prototype.ComponentSelectionPageCallback = function() {
    var widget = gui.currentPageWidget();

    widget.deselectAll();
    widget.selectComponent("qt.qt5.5121.gcc_64");
    widget.selectComponent("qt.qt5.5121.qtcharts");
    widget.selectComponent("qt.qt5.5121.qtdatavis3d");

    gui.clickButton(buttons.NextButton, 3000);
}

Controller.prototype.LicenseAgreementPageCallback = function() {
    gui.currentPageWidget().AcceptLicenseRadioButton.setChecked(true);
    gui.clickButton(buttons.NextButton);
}

Controller.prototype.StartMenuDirectoryPageCallback = function() {
    gui.clickButton(buttons.NextButton);
}

Controller.prototype.ReadyForInstallationPageCallback = function()
{
    gui.clickButton(buttons.NextButton);
}

Controller.prototype.FinishedPageCallback = function() {
var checkBoxForm = gui.currentPageWidget().LaunchQtCreatorCheckBoxForm;
if (checkBoxForm && checkBoxForm.launchQtCreatorCheckBox) {
    checkBoxForm.launchQtCreatorCheckBox.checked = false;
}
    gui.clickButton(buttons.FinishButton);
}
EOF
}

write_qt_script

wget -nc https://download.qt.io/archive/qt/5.12/5.12.1/qt-opensource-linux-x64-5.12.1.run
chmod +x qt-opensource-linux-x64-5.12.1.run
./qt-opensource-linux-x64-5.12.1.run --script $QT_SCRIPT #-platform minimal #--silent

rm -rf qt-opensource-linux-x64-5.12.1.run