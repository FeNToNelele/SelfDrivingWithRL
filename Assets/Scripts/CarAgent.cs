using System;
using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
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
    public float maxSteerAngle = 20.0f;
    public float torqueLimit = 10f;

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
                //if (amount * 10 * torque * Time.deltaTime < 10)
                //{
                //    Debug.Log("SMALL ACCELERATION: " + amount * 10 * torque * Time.deltaTime);
                //}
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

    //public const float MAX_DISTANCE = 15f; //mennyi?
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
        transform.localPosition = new Vector3(0, 0.1f, 0);
        currentGoal = checkpoints[0];
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.localPosition.x);
        sensor.AddObservation(transform.localPosition.y);
        sensor.AddObservation(Vector3.Distance(currentGoal.localPosition, transform.localPosition));

        /*foreach (Transform checkpoint in checkpoints)
        {
            sensor.AddObservation(checkpoint.localPosition.x); //szükséges-e?
            sensor.AddObservation(checkpoint.localPosition.y);

            
        }*/     
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        var actionTaken = actions.ContinuousActions;

        float actionSpeed = (actionTaken[0] + 1) / 2; // [0, +1]
        float actionSteering = actionTaken[1]; // [-1, +1]

        //Debug.Log(String.Format("actionspeed: {0} actionsteering: {1}", actionSpeed, actionSteering));

        Accelerate(actionSpeed);
        Steer(actionSteering);

        //float distance_scaled = Vector3.Distance(currentGoal.localPosition, transform.localPosition) / MAX_DISTANCE;
        //Debug.Log(distance_scaled);

        //AddReward(-distance_scaled / 10); // [0, 0.1]

        AddReward(-0.01f);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> actions = actionsOut.ContinuousActions;

        Debug.Log(String.Format("gas: {0} steer: {1}", actions[0], actions[1]));

        actions[0] = -1;
        actions[1] = 0;

        if (Input.GetKey("w"))
            actions[0] = 1;

        if (Input.GetKey("d"))
            actions[1] = +0.2f;

        if (Input.GetKey("a"))
            actions[1] = -0.2f;
    }


    private void OnCollisionEnter(Collision collision)
    {
        switch (collision.collider.tag)
        {
            case "Wall":
                AddReward(-1);
                EndEpisode();
                break;
            case "Checkpoint1":
                if (!checkpointHit[0])
                {
                    AddReward(1);
                    checkpointHit[0] = true;
                    currentGoal = checkpoints[1];
                }                
                break;
            case "Checkpoint2":
                if (!checkpointHit[1])
                {
                    AddReward(2);
                    checkpointHit[1] = true;
                    currentGoal = checkpoints[2];
                }
                break;
            case "Checkpoint3":
                if (!checkpointHit[2])
                {
                    AddReward(3);
                    checkpointHit[2] = true;
                    currentGoal = checkpoints[3];
                }
                break;
            case "Checkpoint4":
                if (!checkpointHit[3])
                {
                    AddReward(4);
                    checkpointHit[3] = true;
                    currentGoal = checkpoints[4];
                }
                break;
            case "Checkpoint5":
                if (!checkpointHit[4])
                {
                    AddReward(5);
                    checkpointHit[4] = true;
                    currentGoal = checkpoints[5];
                }
                break;
            case "Checkpoint6":
                if (!checkpointHit[5])
                {
                    AddReward(6);
                    checkpointHit[5] = true;
                    currentGoal = checkpoints[6];
                }
                break;
            case "Checkpoint7":
                if (!checkpointHit[6])
                {
                    AddReward(7);
                    checkpointHit[6] = true;
                    currentGoal = checkpoints[7];
                }
                break;
            case "Checkpoint8":
                if (!checkpointHit[7])
                {
                    AddReward(8);
                    checkpointHit[7] = true;
                    currentGoal = checkpoints[8];
                }
                break;
            case "Checkpoint9":
                if (!checkpointHit[8])
                {
                    AddReward(9);
                    checkpointHit[8] = true;
                    currentGoal = checkpoints[9];
                }
                break;
            case "Checkpoint10":
                if (!checkpointHit[10])
                {
                    AddReward(1);
                    checkpointHit[9] = true;
                    currentGoal = checkpoints[10];
                }
                break;
            case "Checkpoint11":
                if (!checkpointHit[10])
                {
                    AddReward(11);
                    checkpointHit[10] = true;
                    currentGoal = checkpoints[0];
                }
                break;
        }
    }

    #endregion
}
