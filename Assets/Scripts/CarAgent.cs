using System;
using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEditor.Experimental.GraphView;
using UnityEngine;
using static CarAgent;

public class CarAgent : Agent
{
    #region CarBehavior

    public enum ControlMode
    {
        KEYBOARD, MODEL
    }

    public enum Axle
    {
        FRONT, REAR
    }

    [Serializable]
    public class Wheel
    {
        public Transform wheelModel;
        public WheelCollider wheelCollider;
        public Axle axle;
    }

    public float torque = 200.0f;
    public float brakeAcceleration = 100.0f;
    public float maxSteerAngle = 45.0f;

    public ControlMode controlMode;
    public Vector3 centerOfMass;
    Rigidbody carRigidBody;
    public List<Wheel> wheels;
    
    float moveInput;
    float steerInput;

    void Start()
    {
        transform.localPosition = new Vector3(0, 0.1f, 0);
        carRigidBody = GetComponent<Rigidbody>();
        carRigidBody.centerOfMass = this.centerOfMass;
        initialRotation = transform.rotation;

        foreach (var wheel in wheels)
        {
            wheel.wheelCollider.contactOffset = 0.1f;
        }
        
    }

    private void FixedUpdate()
    {
        GetInputs();
        
        if (controlMode == ControlMode.KEYBOARD)
        {
            Steer();
            Accelerate();
            Brake();
        }
        
        AnimateWheels();
    }

    void GetInputs()
    {
        moveInput = Input.GetAxis("Vertical");
        steerInput = Input.GetAxis("Horizontal");
    }

    void Accelerate()
    {
        foreach (var wheel in wheels)
        {
            if (wheel.axle == Axle.REAR)
            {
                wheel.wheelCollider.motorTorque = moveInput * 20 * torque * Time.deltaTime;
            }
        }    
    }

    void Accelerate(float amount)
    {
        foreach (var wheel in wheels)
        {
            if (wheel.axle == Axle.REAR)
            {
                wheel.wheelCollider.motorTorque = amount * 30 * torque * Time.deltaTime;
            }
        }
    }

    void AnimateWheels()
    {
        foreach (var wheel in wheels)
        {
                AnimateWheels(wheel.wheelModel, wheel.wheelCollider);
                AnimateWheels(wheel.wheelModel, wheel.wheelCollider);            
        }
    }

    private void AnimateWheels(Transform wheelModel, WheelCollider wheelCollider)
    {
        Vector3 _position = transform.position;
        Quaternion _quaternion = transform.rotation;

        wheelCollider.GetWorldPose(out _position, out _quaternion);

        wheelModel.position = _position;
        wheelModel.rotation = _quaternion;
    }

    void Steer()
    {
        foreach (var wheel in wheels)
        {
            if (wheel.axle == Axle.FRONT)
            {
                var steerAngle = steerInput * maxSteerAngle;
                wheel.wheelCollider.steerAngle = Mathf.Lerp(wheel.wheelCollider.steerAngle, steerAngle, 0.3f);
            }
        }
    }
    
    void Steer(float steerAngle)
    {
        foreach (var wheel in wheels)
        {
            if (wheel.axle == Axle.FRONT)
            {
                wheel.wheelCollider.steerAngle = Mathf.Lerp(wheel.wheelCollider.steerAngle,
                    steerAngle * maxSteerAngle, 0.6f);
            }
        }
    }

    void Brake()
    {
        if (Input.GetKey(KeyCode.S) || moveInput == 0)
        {
            foreach (var wheel in wheels)
            {
                wheel.wheelCollider.brakeTorque = 600 * brakeAcceleration * Time.deltaTime;
            }
        }
        else
        {
            foreach (var wheel in wheels)
            {
                wheel.wheelCollider.brakeTorque = 0;
            }
        }
    }

    void ResetCar()
    {
        transform.rotation = initialRotation;
        transform.localPosition = new Vector3(0, 0.1f, 0);

        foreach (var wheel in wheels)
        {
            wheel.wheelCollider.motorTorque = 0;
            wheel.wheelCollider.steerAngle = 0;
        }

        carRigidBody.velocity = Vector3.zero;
        carRigidBody.angularVelocity = Vector3.zero;
    }

    #endregion

    #region RL

    Quaternion initialRotation;
    public List<Transform> checkpoints;
    public List<bool> checkpointHit = new List<bool>()
    {
        false, false, false, false, false, false, false, false, false, false, false
    };

    public Transform currentGoal;


    public override void OnEpisodeBegin()
    {
        ResetCar();
        currentGoal = checkpoints[0];

        for (int i = 0; i < checkpointHit.Count; i++)
        {
            checkpointHit[i] = false;
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.localPosition.x);
        sensor.AddObservation(transform.localPosition.y);
        sensor.AddObservation(Vector3.Distance(currentGoal.localPosition, transform.localPosition));   
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        var actionTaken = actions.ContinuousActions;

        float actionSpeed = (actionTaken[0] + 1) / 2;
        float actionSteering = actionTaken[1];

        Accelerate(actionSpeed);
        Steer(actionSteering);

        //AddReward(-distance_scaled / 10); // [0, 0.1]

        AddReward(-0.005f);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> actions = actionsOut.ContinuousActions;

        actions[0] = -1;
        actions[1] = 0;

        if (Input.GetKey("w"))
            actions[0] = 1;

        if (Input.GetKey("d"))
            actions[1] = +0.2f;

        if (Input.GetKey("a"))
            actions[1] = -0.2f;
    }


    private Dictionary<string, (int reward, int checkpointIndex)> cpsWithRewards = new Dictionary<string, (int, int)> {
        { "Wall", (-1, -1) },
        { "Checkpoint1", (1, 0) },
        { "Checkpoint2", (1, 1) },
        { "Checkpoint3", (1, 2) },
        { "Checkpoint4", (1, 3) },
        { "Checkpoint5", (1, 4) },
        { "Checkpoint6", (1, 5) },
        { "Checkpoint7", (1, 6) },
        { "Checkpoint8", (3, 7) },
        { "Checkpoint9", (1, 8) },
        { "Checkpoint10", (1, 9) },
        { "Checkpoint11", (1, 10) }
    };

    private void OnCollisionEnter(Collision collision)
    {
        string tag = collision.collider.tag;

        (int reward, int checkpointIndex) = cpsWithRewards[tag];

        AddReward(reward);


        if (checkpointIndex == -1)
        {
            EndEpisode();
        }


        if ((checkpointIndex + 1) == checkpoints.Count)
        {
            currentGoal = checkpoints[0];
        }
        else currentGoal = checkpoints[(checkpointIndex + 1)];
    }

    #endregion
}
