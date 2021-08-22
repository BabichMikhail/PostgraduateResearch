using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class ExperimentsController : MonoBehaviour {
    public List<GameObject> samples;
    public List<float> maxSpeeds;
    public List<float> yRotations;
    public string experimentsStoreFolder;

    private CommonSettings commonSettings;

    public void Awake() {
        var runExperimentsButton = GameObject.Find("RunExperimentsButton").GetComponent<Button>();
        runExperimentsButton.onClick.AddListener(RunExperiments);

        commonSettings = GameObject.Find("Settings").GetComponent<CommonSettings>();
    }

    private void RunExperiments() {
        foreach (var sample in samples) {
            sample.SetActive(false);
        }

        foreach (var sample in samples) {
            foreach (var maxSpeed in maxSpeeds) {
                foreach (var yRotation in yRotations) {
                    var copy = Instantiate(sample, transform);
                    copy.SetActive(true);

                    var controller = copy.GetComponent<ExtractVertices>();
                    controller.maxPaintRobotSpeed = maxSpeed;
                    controller.needRunExperiment = true;
                    controller.yRotation = yRotation;
                    controller.experimentStoreFolder = experimentsStoreFolder;

                    copy.name = $"_{sample.name}_s{controller.paintSpeed}_ms{controller.maxPaintRobotSpeed}_rot{controller.yRotation}";
                }
            }
        }
    }
}
