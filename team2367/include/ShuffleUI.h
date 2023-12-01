#pragma once

#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>
#include <vector>
#include "UIWidget.h"
#include <iostream>

// A static class for making Shuffleboard easier to use
class ShuffleUI {
  private:
    // static vector to store info on created widgets, using the UIWidget class.
    static std::vector<UIWidget *> widgetList;

    /**
     * Adds a UIWidget to a list of created widgets
     *
     * @param widget pointer to a UIWidget
     */
    static void AddEntry(UIWidget *widget);


  public:
    /**
     * Creates a new widget, or updates an existing widget's value.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @param value Widget's value
     * @return pointer to the widget's entry
     */
    static nt::GenericEntry *MakeWidget(std::string name, std::string tab, int value);
    
    /**
     * Creates a new widget, or updates an existing widget's value.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @param value Widget's value
     * @return pointer to the widget's entry
     */
    static nt::GenericEntry *MakeWidget(std::string name, std::string tab, double value);
    
    /**
     * Creates a new widget, or updates an existing widget's value.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @param value Widget's value
     * @return pointer to the widget's entry
     */
    static nt::GenericEntry *MakeWidget(std::string name, std::string tab, float value);

    /**
     * Creates a new widget, or updates an existing widget's value.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @param value Widget's value
     * @return pointer to the widget's entry
     */
    static nt::GenericEntry *MakeWidget(std::string name, std::string tab, bool value);
    
    /**
     * Creates a new widget with a slider.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @param min Slider lower bound
     * @param max Slider upper bound
     * @param defaultVal Slider's default value
     * @return pointer to the widget's entry
     */
    static nt::GenericEntry *MakeSlider(std::string name, std::string tab, double min, double max, double defaultVal);

    /**
     * Finds an existing widget.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @return pointer to the widget's entry, or NULL if widget does not exist
     */
    static nt::GenericEntry *GetEntry(std::string name, std::string tab);

    /**
     * Creates a new widget with a graph, or updates an existing graph's value.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @param value Widget's value
     * @return pointer to the widget's entry
     */
    static nt::GenericEntry *MakeGraph(std::string name, std::string tab, double value);

    /**
     * Creates a new widget with a button.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @param defaultState Widget's default state
     * @return pointer to the widget's entry
     */
    static nt::GenericEntry *MakeButton(std::string name, std::string tab, bool defaultState);

    /**
     * Prints the names of all widgets.
     */
    static void PrintWidgetList();
};



std::vector<UIWidget *> ShuffleUI::widgetList;

// makes a widget and returns a pointer to its entry. If widget with that name already exists, updates its value and returns that entry's pointer instead
nt::GenericEntry *ShuffleUI::MakeWidget(std::string name, std::string tab, int value=0) {
    if (GetEntry(name, tab) != NULL) { 
        //std::cout << "widget already exists" << std::endl;
        try {
            GetEntry(name, tab)->SetInteger(value);
        } catch(...) {}
        return GetEntry(name, tab);
    }
    nt::GenericEntry *entry = frc::Shuffleboard::GetTab(tab).Add(name, value).GetEntry();
    UIWidget *widget = new UIWidget(entry, tab, name);
    ShuffleUI::AddEntry(widget);
    return entry;
}

// makes a widget and returns a pointer to its entry. If widget with that name already exists, updates its value and returns that entry's pointer instead
nt::GenericEntry *ShuffleUI::MakeWidget(std::string name, std::string tab, double value=0.0) {
    if (GetEntry(name, tab) != NULL) { 
        //std::cout << "widget already exists" << std::endl;
        try {
            GetEntry(name, tab)->SetDouble(value);
        } catch(...) {}
        return GetEntry(name, tab);
    }
    nt::GenericEntry *entry = frc::Shuffleboard::GetTab(tab).Add(name, value).GetEntry();
    UIWidget *widget = new UIWidget(entry, tab, name);
    ShuffleUI::AddEntry(widget);
    return entry;
}

// makes a widget and returns a pointer to its entry. If widget with that name already exists, updates its value and returns that entry's pointer instead
nt::GenericEntry *ShuffleUI::MakeWidget(std::string name, std::string tab, float value) {
    if (GetEntry(name, tab) != NULL) { 
        //std::cout << "widget already exists" << std::endl;
        try {
            GetEntry(name, tab)->SetFloat(value);
        } catch(...) {}
        return GetEntry(name, tab);
    }
    nt::GenericEntry *entry = frc::Shuffleboard::GetTab(tab).Add(name, value).GetEntry();
    UIWidget *widget = new UIWidget(entry, tab, name);
    ShuffleUI::AddEntry(widget);
    return entry;
}

nt::GenericEntry *ShuffleUI::MakeWidget(std::string name, std::string tab, bool value) {
    if (GetEntry(name, tab) != NULL) { 
        //std::cout << "widget already exists" << std::endl;
        try {
            GetEntry(name, tab)->SetBoolean(value);
        } catch(...) {}
        return GetEntry(name, tab);
    }
    nt::GenericEntry *entry = frc::Shuffleboard::GetTab(tab).Add(name, value)
        .WithWidget(frc::BuiltInWidgets::kBooleanBox)
        .GetEntry();
    UIWidget *widget = new UIWidget(entry, tab, name);
    ShuffleUI::AddEntry(widget);
    return entry;
}

// makes a widget with number slider and returns a pointer to its entry. If widget with that name already exists, returns that entry's pointer instead
nt::GenericEntry *ShuffleUI::MakeSlider(std::string name, std::string tab, double min, double max, double defaultVal=0.0) {
    if (GetEntry(name, tab) != NULL) { 
        //std::cout << "widget already exists" << std::endl;
        return GetEntry(name, tab);
    }
    nt::GenericEntry *entry = frc::Shuffleboard::GetTab(tab).Add(name, defaultVal)
        .WithWidget(frc::BuiltInWidgets::kNumberSlider)
        .WithProperties({
            {"min", nt::Value::MakeDouble(min)},
            {"max", nt::Value::MakeDouble(max)}
        })
        .GetEntry();
    UIWidget *widget = new UIWidget(entry, tab, name);
    ShuffleUI::AddEntry(widget);
    return entry;
} 

void ShuffleUI::AddEntry(UIWidget *widget) {
    ShuffleUI::widgetList.push_back(widget);
}

// finds an Entry by name & tab, returns NULL if it doesn't exist
nt::GenericEntry *ShuffleUI::GetEntry(std::string name, std::string tab) {
    for (UIWidget *w: ShuffleUI::widgetList) {
        if (w->GetName() == name && w->GetTab() == tab) {
            return w->GetEntry();
        }
    }
    return NULL;
}

//prints the name of all widgets in the widget list
void ShuffleUI::PrintWidgetList() {
    for (UIWidget *w: ShuffleUI::widgetList) {
        std::cout << w->GetName() << std::endl;
    }
}

// makes a widget with a graph and returns a pointer to its entry. If widget with that name already exists, returns that entry's pointer instead
nt::GenericEntry *ShuffleUI::MakeGraph(std::string name, std::string tab, double value) {
    if (GetEntry(name, tab) != NULL) { 
        //std::cout << "widget already exists" << std::endl;
        try {
            GetEntry(name, tab)->SetDouble(value);
        } catch(...) {}
        return GetEntry(name, tab);
    }
    nt::GenericEntry *entry = frc::Shuffleboard::GetTab(tab).Add(name, value)
        .WithWidget(frc::BuiltInWidgets::kGraph)
        .GetEntry();
    UIWidget *widget = new UIWidget(entry, tab, name);
    ShuffleUI::AddEntry(widget);
    return entry;
}

nt::GenericEntry *ShuffleUI::MakeButton(std::string name, std::string tab, bool defaultState) {
    if (GetEntry(name, tab) != NULL) { 
        return GetEntry(name, tab);
    }
    nt::GenericEntry *entry = frc::Shuffleboard::GetTab(tab).Add(name, defaultState)
        .WithWidget(frc::BuiltInWidgets::kToggleButton)
        .GetEntry();
    UIWidget *widget = new UIWidget(entry, tab, name);
    ShuffleUI::AddEntry(widget);
    return entry;
}