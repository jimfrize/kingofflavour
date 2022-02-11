#include "plugin.hpp"
#include <dsp/filter.hpp>

struct MSG : Module
{

	rack::dsp::TRCFilter<float> LEFT_HPF;
	rack::dsp::TRCFilter<float> RIGHT_HPF;
	rack::dsp::TRCFilter<float> LEFT_HPF2;
	rack::dsp::TRCFilter<float> RIGHT_HPF2;
	
	enum ParamId
	{
		DRIVE_PARAM,
		CUTOFF_PARAM,
		MIX_PARAM,
		COMP_PARAM,
		PARAMS_LEN
	};
	enum InputId
	{
		LEFT_INPUT,
		RIGHT_INPUT,
		DRIVE_INPUT,
		CUTOFF_INPUT,
		MIX_INPUT,
		COMP_INPUT,
		INPUTS_LEN
	};
	enum OutputId
	{
		LEFT_OUTPUT,
		RIGHT_OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId
	{
		LIGHTS_LEN
	};

	MSG()
	{
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

		configParam(DRIVE_PARAM, std::log2(1.f), std::log2(10.f), std::log2(1.f), "drive", "", 2.f);
		configParam(CUTOFF_PARAM, std::log2(20000.f), std::log2(20.f), std::log2(20000.f), "cutoff", "Hz", 2.f);
		configParam(MIX_PARAM, 0.f, 1.f, 0.5f, "mix");
		configParam(COMP_PARAM, 0.f, 2.f, 1.f, "compenstaion");

		configInput(LEFT_INPUT, "left input");
		configInput(RIGHT_INPUT, "right input");
		configInput(DRIVE_INPUT, "cutoff CV input");
		configInput(CUTOFF_INPUT, "bandwidth CV input");
		configInput(MIX_INPUT, "mix CV input");
		configInput(COMP_INPUT, "compensation CV input");

		configOutput(LEFT_OUTPUT, "left output");
		configOutput(RIGHT_OUTPUT, "right output");
		configBypass(LEFT_INPUT, LEFT_OUTPUT);
		configBypass(RIGHT_INPUT, RIGHT_OUTPUT);
	}

	float drive_cv = 1.f;
	float cutoff_cv = 1.f;
	float mix_cv = 1.f;
	float comp_cv = 1.f;

	float drive = 1.f;
	float cutoff = 20000.f;
	float range = 0.f;
	float mix = 0.5;
	float comp = 0.5f;

	void process(const ProcessArgs& args) override
	{
		if( inputs[LEFT_INPUT].isConnected() || inputs[RIGHT_INPUT].isConnected() ) // check if either input is connected
		{
			/////////////////////////////////////////
			// get & clamp any connected CV inputs //
			/////////////////////////////////////////

			if(inputs[DRIVE_INPUT].isConnected())
			{
				drive_cv = inputs[DRIVE_INPUT].getVoltage() / 10.f;
				drive_cv = clamp(drive_cv, 0.f, 1.f);
			}
			else{ drive_cv = 1.f; }
			
			if(inputs[CUTOFF_INPUT].isConnected())
			{
				cutoff_cv = inputs[CUTOFF_INPUT].getVoltage() / 10.f;
				cutoff_cv = clamp(cutoff_cv, 0.f, 1.f);
			}
			else{ cutoff_cv = 1.f; }
			
			if(inputs[MIX_INPUT].isConnected())
			{
				mix_cv = inputs[MIX_INPUT].getVoltage() / 10.f;
				mix_cv = clamp(mix_cv, 0.f, 1.f);
			}
			else{ mix_cv = 1.f; }

			if(inputs[COMP_INPUT].isConnected())
			{
				comp_cv = inputs[COMP_INPUT].getVoltage() / 10.f;
				comp_cv = clamp(comp_cv, 0.f, 1.f);
			}
			else{ comp_cv = 1.f; }

			///////////////////////////////////////
			// get & calculate shared parameters //
			///////////////////////////////////////

			drive = std::pow( 2.f, params[DRIVE_PARAM].getValue() ) * drive_cv;
			range = ( 20000.f - std::pow( 2.f, params[CUTOFF_PARAM].getValue() ) ) * cutoff_cv;
			cutoff = 20000.f - range;
			mix = params[MIX_PARAM].getValue() * mix_cv;
			comp = params[COMP_PARAM].getValue() * comp_cv;

			///////////////////////////////////
			// calculate and set left output //
			///////////////////////////////////

			if( inputs[LEFT_INPUT].isConnected() && outputs[LEFT_OUTPUT].isConnected() )
			{
				float left = inputs[LEFT_INPUT].getVoltage();
				float dist = atan(left / 5.f * drive) * 10.f; // calculate distortion

				//*********//
				// filters //
				//*********//

				LEFT_HPF.setCutoffFreq(cutoff / args.sampleRate);
				LEFT_HPF2.setCutoffFreq(cutoff / args.sampleRate);
				LEFT_HPF.process(dist);
				dist = LEFT_HPF.highpass();
				LEFT_HPF2.process(dist);
				dist = LEFT_HPF2.highpass();

				float output = (dist * mix) + ( (1.f - mix) * (left * 2.f)); // calculate output
				outputs[LEFT_OUTPUT].setVoltage(output * comp);
			}

			////////////////////////////////////
			// calculate and set right output //
			////////////////////////////////////

			if( inputs[RIGHT_INPUT].isConnected() && outputs[RIGHT_OUTPUT].isConnected() )
			{
				float right = inputs[RIGHT_INPUT].getVoltage();
				float dist = atan(right / 5.f * drive) * 10.f; // calculate distortion

				//*********//
				// filters //
				//*********//

				RIGHT_HPF.setCutoffFreq(cutoff / args.sampleRate);
				RIGHT_HPF2.setCutoffFreq(cutoff / args.sampleRate);
				RIGHT_HPF.process(dist);
				dist = RIGHT_HPF.highpass();
				RIGHT_HPF2.process(dist);
				dist = RIGHT_HPF2.highpass();

				float output = (dist * mix) + ( (1.f - mix) * (right * 2.f)); // calculate output
				outputs[RIGHT_OUTPUT].setVoltage(output * comp);
			}
		}
	}
};

struct JACKPort : app::SvgPort
{
    JACKPort()
	{
        setSvg(Svg::load(asset::plugin(pluginInstance, "res/jack.svg")));
		shadow->opacity = 0.0;
    }
};

struct MSGWidget : ModuleWidget
{
	MSGWidget(MSG* module)
	{
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/msg.svg")));

		addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(6.08f, 34.f)), module, MSG::DRIVE_PARAM));
		addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(14.24f, 50.f)), module, MSG::CUTOFF_PARAM));
		addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(6.08f, 66.f)), module, MSG::MIX_PARAM));
		addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(14.24f, 82.f)), module, MSG::COMP_PARAM));

		addInput(createInputCentered<JACKPort>(mm2px(Vec(14.24f, 34.f)), module, MSG::DRIVE_INPUT));
		addInput(createInputCentered<JACKPort>(mm2px(Vec(6.08f, 50.f)), module, MSG::CUTOFF_INPUT));
		addInput(createInputCentered<JACKPort>(mm2px(Vec(14.24f, 66.f)), module, MSG::MIX_INPUT));
		addInput(createInputCentered<JACKPort>(mm2px(Vec(6.08f, 82.f)), module, MSG::COMP_INPUT));


		addInput(createInputCentered<JACKPort>(mm2px(Vec(6.08f, 94.f)), module, MSG::LEFT_INPUT));
		addInput(createInputCentered<JACKPort>(mm2px(Vec(14.24f, 94.f)), module, MSG::RIGHT_INPUT));

		addOutput(createOutputCentered<JACKPort>(mm2px(Vec(6.08f, 104.f)), module, MSG::LEFT_OUTPUT));
		addOutput(createOutputCentered<JACKPort>(mm2px(Vec(14.24f, 104.f)), module, MSG::RIGHT_OUTPUT));
	}
};


Model* modelMSG = createModel<MSG, MSGWidget>("MSG");