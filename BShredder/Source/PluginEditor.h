/*
  ==============================================================================

    This file was auto-generated!

    It contains the basic framework code for a JUCE plugin editor.

  ==============================================================================
*/

#pragma once

#include "JuceHeader.h"
#include "PluginProcessor.h"

#include <ATKJUCEComponents/JUCE/ImageLookAndFeel.h>
#include <ATKJUCEComponents/JUCE/Slider.h>

//==============================================================================
/**
 */
class BShredderAudioProcessorEditor: public juce::AudioProcessorEditor
{
public:
  BShredderAudioProcessorEditor(BShredderAudioProcessor& p, juce::AudioProcessorValueTreeState& paramState);
  ~BShredderAudioProcessorEditor();

  //==============================================================================
  void paint(juce::Graphics&) override;
  void resized() override;

private:
  // This reference is provided as a quick way for your editor to
  // access the processor object that created it.
  BShredderAudioProcessor& processor;
  juce::AudioProcessorValueTreeState& paramState;

  ATK::juce::ImageLookAndFeel knob;
  juce::Image bckgndImage;

  ATK::juce::SliderComponent distLevel;
  ATK::juce::SliderComponent lowLevel;
  ATK::juce::SliderComponent highLevel;
  ATK::juce::SliderComponent midLevel;
  ATK::juce::SliderComponent midFreq;
};
