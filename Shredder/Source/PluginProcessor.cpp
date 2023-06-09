/*
  ==============================================================================

    This file was auto-generated!

    It contains the basic framework code for a JUCE plugin processor.

  ==============================================================================
*/

#include "PluginProcessor.h"
#include "PluginEditor.h"
#include "static_elements.h"

//==============================================================================
ShredderAudioProcessor::ShredderAudioProcessor()
  :
#ifndef JucePlugin_PreferredChannelConfigurations
  AudioProcessor(BusesProperties()
#  if !JucePlugin_IsMidiEffect
#    if !JucePlugin_IsSynth
                     .withInput("Input", AudioChannelSet::stereo(), true)
#    endif
                     .withOutput("Output", AudioChannelSet::stereo(), true)
#  endif
          )
  ,
#endif
  inFilter(nullptr, 1, 0, false)
  , highPassFilter(Shredder::createStaticFilter_stage1())
  , oversamplingFilter(1)
  , preDistortionToneShapingFilter(Shredder::createStaticFilter_stage2())
  , bandPassFilter(Shredder::createStaticFilter_stage3())
  , distLevelFilter(Shredder::createStaticFilter_stage4())
  , distFilter(Shredder::createStaticFilter_stage5())
  , postDistortionToneShapingFilter(Shredder::createStaticFilter_stage6())
  , lowpassFilter(1)
  , decimationFilter(1)
  , DCFilter(1)
  , lowToneControlFilter(1)
  , highToneControlFilter(1)
  , sweepableMidToneControlFilter(1)
  , outFilter(nullptr, 1, 0, false)
  , parameters(*this,
        nullptr,
        juce::Identifier("ATKShredder"),
        {std::make_unique<juce::AudioParameterFloat>("distLevel", "Distortion Level", 0.f, 100.f, 50.f),
            std::make_unique<juce::AudioParameterFloat>("lowLevel", "Low Freq Level", -20.0f, 20.0f, .0f),
            std::make_unique<juce::AudioParameterFloat>("highLevel", "High Freq Level", -20.0f, 20.0f, .0f),
            std::make_unique<juce::AudioParameterFloat>("midLevel", "Mid Freq Level", -15.f, 15.0f, .0f),
            std::make_unique<juce::AudioParameterFloat>("midFreq", "Mid Freq", 240.f, 6300.f, 1000.f),
            std::make_unique<juce::AudioParameterFloat>("lowQ", "Low Q", 1.f, 4.f, 3.1f),
            std::make_unique<juce::AudioParameterFloat>("highQ", "High Q", .1f, .5f, 0.25f),
            std::make_unique<juce::AudioParameterFloat>("midQ", "MidQ", 0.5f, 4.f, 1.f)})
{
  highPassFilter->set_input_port(highPassFilter->find_input_pin("vin"), &inFilter, 0);
  oversamplingFilter.set_input_port(0, highPassFilter.get(), highPassFilter->find_dynamic_pin("vout"));
  preDistortionToneShapingFilter->set_input_port(
      preDistortionToneShapingFilter->find_input_pin("vin"), &oversamplingFilter, 0);
  bandPassFilter->set_input_port(bandPassFilter->find_input_pin("vin"),
      preDistortionToneShapingFilter.get(),
      preDistortionToneShapingFilter->find_dynamic_pin("vout"));
  distLevelFilter->set_input_port(
      distLevelFilter->find_input_pin("vin"), bandPassFilter.get(), bandPassFilter->find_dynamic_pin("vout"));
  distFilter->set_input_port(
      distFilter->find_input_pin("vin"), distLevelFilter.get(), distLevelFilter->find_dynamic_pin("vout"));
  postDistortionToneShapingFilter->set_input_port(
      postDistortionToneShapingFilter->find_input_pin("vin"), distFilter.get(), distFilter->find_dynamic_pin("vout"));
  lowpassFilter.set_input_port(
      0, postDistortionToneShapingFilter.get(), postDistortionToneShapingFilter->find_dynamic_pin("vout"));
  decimationFilter.set_input_port(0, &lowpassFilter, 0);
  DCFilter.set_input_port(0, &decimationFilter, 0);
  lowToneControlFilter.set_input_port(0, &DCFilter, 0);
  highToneControlFilter.set_input_port(0, &lowToneControlFilter, 0);
  sweepableMidToneControlFilter.set_input_port(0, &highToneControlFilter, 0);
  outFilter.set_input_port(0, &sweepableMidToneControlFilter, 0);

  lowpassFilter.set_cut_frequency(20000);
  lowpassFilter.set_order(6);
  DCFilter.set_cut_frequency(1);
  DCFilter.set_order(2);
}

ShredderAudioProcessor::~ShredderAudioProcessor() = default;

//==============================================================================
const juce::String ShredderAudioProcessor::getName() const
{
  return JucePlugin_Name;
}

bool ShredderAudioProcessor::acceptsMidi() const
{
#if JucePlugin_WantsMidiInput
  return true;
#else
  return false;
#endif
}

bool ShredderAudioProcessor::producesMidi() const
{
#if JucePlugin_ProducesMidiOutput
  return true;
#else
  return false;
#endif
}

bool ShredderAudioProcessor::isMidiEffect() const
{
#if JucePlugin_IsMidiEffect
  return true;
#else
  return false;
#endif
}

double ShredderAudioProcessor::getTailLengthSeconds() const
{
  return 0.0;
}

int ShredderAudioProcessor::getNumPrograms()
{
  return 2;
}

int ShredderAudioProcessor::getCurrentProgram()
{
  return lastParameterSet;
}

void ShredderAudioProcessor::setCurrentProgram(int index)
{
  if(index != lastParameterSet)
  {
    lastParameterSet = index;
    if(index == 0)
    {
      const char* preset0
          = "<Shredder><PARAM id=\"distLevel\" value=\"0\" /><PARAM id=\"lowLevel\" value=\"0\" /><PARAM id=\"highLevel\" "
            "value=\"0\" /> <PARAM id=\"midLevel\" value=\"0\" /><PARAM id=\"midFreq\" value=\"1000\" /></Shredder>";
      juce::XmlDocument doc(preset0);

      auto el = doc.getDocumentElement();
      parameters.state = juce::ValueTree::fromXml(*el);
    }
    else if(index == 1)
    {
      const char* preset1 = "<Shredder><PARAM id=\"distLevel\" value=\"100\" /><PARAM id=\"lowLevel\" value=\"20\" /><PARAM "
                            "id=\"highLevel\" value=\"20\" /> <PARAM id=\"midLevel\" value=\"15\" /><PARAM "
                            "id=\"midFreq\" value=\"1000\" /></Shredder>";
      juce::XmlDocument doc(preset1);

      auto el = doc.getDocumentElement();
      parameters.state = juce::ValueTree::fromXml(*el);
    }
  }
}

const juce::String ShredderAudioProcessor::getProgramName(int index)
{
  if(index == 0)
  {
    return "Minimum distortion";
  }
  if(index == 0)
  {
    return "Maximum damage";
  }
  return {};
}

void ShredderAudioProcessor::changeProgramName(int index, const juce::String& newName)
{
}

//==============================================================================
void ShredderAudioProcessor::prepareToPlay(double dbSampleRate, int samplesPerBlock)
{
  sampleRate = std::lround(dbSampleRate);

  if(sampleRate != inFilter.get_output_sampling_rate())
  {
    inFilter.set_input_sampling_rate(sampleRate);
    inFilter.set_output_sampling_rate(sampleRate);
    highPassFilter->set_input_sampling_rate(sampleRate);
    highPassFilter->set_output_sampling_rate(sampleRate);
    oversamplingFilter.set_input_sampling_rate(sampleRate);
    oversamplingFilter.set_output_sampling_rate(sampleRate * OVERSAMPLING);
    preDistortionToneShapingFilter->set_input_sampling_rate(sampleRate * OVERSAMPLING);
    preDistortionToneShapingFilter->set_output_sampling_rate(sampleRate * OVERSAMPLING);
    bandPassFilter->set_input_sampling_rate(sampleRate * OVERSAMPLING);
    bandPassFilter->set_output_sampling_rate(sampleRate * OVERSAMPLING);
    distLevelFilter->set_input_sampling_rate(sampleRate * OVERSAMPLING);
    distLevelFilter->set_output_sampling_rate(sampleRate * OVERSAMPLING);
    distFilter->set_input_sampling_rate(sampleRate * OVERSAMPLING);
    distFilter->set_output_sampling_rate(sampleRate * OVERSAMPLING);
    postDistortionToneShapingFilter->set_input_sampling_rate(sampleRate * OVERSAMPLING);
    postDistortionToneShapingFilter->set_output_sampling_rate(sampleRate * OVERSAMPLING);
    lowpassFilter.set_input_sampling_rate(sampleRate * OVERSAMPLING);
    lowpassFilter.set_output_sampling_rate(sampleRate * OVERSAMPLING);
    decimationFilter.set_input_sampling_rate(sampleRate * OVERSAMPLING);
    decimationFilter.set_output_sampling_rate(sampleRate);
    DCFilter.set_input_sampling_rate(sampleRate);
    DCFilter.set_output_sampling_rate(sampleRate);
    lowToneControlFilter.set_input_sampling_rate(sampleRate);
    lowToneControlFilter.set_output_sampling_rate(sampleRate);
    highToneControlFilter.set_input_sampling_rate(sampleRate);
    highToneControlFilter.set_output_sampling_rate(sampleRate);
    sweepableMidToneControlFilter.set_input_sampling_rate(sampleRate);
    sweepableMidToneControlFilter.set_output_sampling_rate(sampleRate);
    outFilter.set_input_sampling_rate(sampleRate);
    outFilter.set_output_sampling_rate(sampleRate);

    lowToneControlFilter.set_cut_frequency(100);
    highToneControlFilter.set_cut_frequency(10000);
  }
  outFilter.dryrun(samplesPerBlock);
}

void ShredderAudioProcessor::releaseResources()
{
  // When playback stops, you can use this as an opportunity to free up any
  // spare memory, etc.
}

#ifndef JucePlugin_PreferredChannelConfigurations
bool ShredderAudioProcessor::isBusesLayoutSupported(const juce::BusesLayout& layouts) const
{
#  if JucePlugin_IsMidiEffect
  ignoreUnused(layouts);
  return true;
#  else
  // This is the place where you check if the layout is supported.
  if(layouts.getMainOutputChannelSet() != AudioChannelSet::stereo())
    return false;

    // This checks if the input layout matches the output layout
#    if !JucePlugin_IsSynth
  if(layouts.getMainOutputChannelSet() != layouts.getMainInputChannelSet())
    return false;
#    endif

  return true;
#  endif
}
#endif

void ShredderAudioProcessor::processBlock(juce::AudioSampleBuffer& buffer, juce::MidiBuffer& midiMessages)
{
  if(*parameters.getRawParameterValue("distLevel") != old_distLevel)
  {
    old_distLevel = *parameters.getRawParameterValue("distLevel");
    distLevelFilter->set_parameter(0, old_distLevel * .99 / 100 + .05);
  }
  if(*parameters.getRawParameterValue("lowLevel") != old_lowLevel)
  {
    old_lowLevel = *parameters.getRawParameterValue("lowLevel");
    lowToneControlFilter.set_gain(std::pow(10, old_lowLevel / 40));
  }
  if(*parameters.getRawParameterValue("highLevel") != old_highLevel)
  {
    old_highLevel = *parameters.getRawParameterValue("highLevel");
    highToneControlFilter.set_gain(std::pow(10, old_highLevel / 40));
  }
  if(*parameters.getRawParameterValue("midLevel") != old_midLevel)
  {
    old_midLevel = *parameters.getRawParameterValue("midLevel");
    sweepableMidToneControlFilter.set_gain(std::pow(10, old_midLevel / 40));
  }
  if(*parameters.getRawParameterValue("midFreq") != old_midFreq)
  {
    old_midFreq = *parameters.getRawParameterValue("midFreq");
    sweepableMidToneControlFilter.set_cut_frequency(old_midFreq);
  }
  if(*parameters.getRawParameterValue("lowQ") != old_lowQ)
  {
    old_lowQ = *parameters.getRawParameterValue("lowQ");
    lowToneControlFilter.set_Q(old_lowQ);
  }
  if(*parameters.getRawParameterValue("highQ") != old_highQ)
  {
    old_highQ = *parameters.getRawParameterValue("highQ");
    highToneControlFilter.set_Q(old_highQ);
  }
  if(*parameters.getRawParameterValue("midQ") != old_midQ)
  {
    old_midQ = *parameters.getRawParameterValue("midQ");
    sweepableMidToneControlFilter.set_Q(old_midQ);
  }

  const int totalNumInputChannels = getTotalNumInputChannels();
  const int totalNumOutputChannels = getTotalNumOutputChannels();

  assert(totalNumInputChannels == totalNumOutputChannels);
  assert(totalNumOutputChannels == 1);

  inFilter.set_pointer(buffer.getReadPointer(0), buffer.getNumSamples());
  outFilter.set_pointer(buffer.getWritePointer(0), buffer.getNumSamples());

  outFilter.process(buffer.getNumSamples());
}

//==============================================================================
bool ShredderAudioProcessor::hasEditor() const
{
  return true; // (change this to false if you choose to not supply an editor)
}

juce::AudioProcessorEditor* ShredderAudioProcessor::createEditor()
{
  return new ShredderAudioProcessorEditor(*this, parameters);
}

//==============================================================================
void ShredderAudioProcessor::getStateInformation(juce::MemoryBlock& destData)
{
  auto state = parameters.copyState();
  std::unique_ptr<juce::XmlElement> xml(state.createXml());
  xml->setAttribute("version", "0");
  copyXmlToBinary(*xml, destData);
}

void ShredderAudioProcessor::setStateInformation(const void* data, int sizeInBytes)
{
  std::unique_ptr<juce::XmlElement> xmlState(getXmlFromBinary(data, sizeInBytes));

  if(xmlState.get() != nullptr)
  {
    if(xmlState->hasTagName(parameters.state.getType()))
    {
      if(xmlState->getStringAttribute("version") == "0")
      {
        parameters.replaceState(juce::ValueTree::fromXml(*xmlState));
      }
    }
  }
}

//==============================================================================
// This creates new instances of the plugin..
juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
  return new ShredderAudioProcessor();
}
