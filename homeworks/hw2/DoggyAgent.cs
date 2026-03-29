using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections;
using System;
using Random = UnityEngine.Random;
using UnityEngine.InputSystem;

public class DoggyAgent : Agent
{
    [Header("Сервоприводы")]
    public ArticulationBody[] legs;

    [Header("Скорость работы сервоприводов")]
    public float servoSpeed;

    [Header("Тело")]
    public ArticulationBody body;
    private Vector3 defPos;
    private Quaternion defRot;
    public float strenghtMove;
    public float fallThreshold=0.3f;
    public float heightThreshold;

    [Header("Куб (цель)")]
    public GameObject cube;

    [Header("Сенсоры")]
    public Unity.MLAgentsExamples.GroundContact[] groundContacts;

    private float distToTarget = 0f;
    public float notFallingRewardMultiplier = 0.0001f;
    private float energyPenalty = 1.0f;
    public float fallingReward = -10f;
    [Header("Наклон относительно земли")]
    public float standingReward = 1f;
    public float notStandingReward = -0.5f;
    [Header("Высота")]
    public float heightRewardMultiplier = 0.0001f;
    public float negativeHeightRewardMultiplier = -0.1f;
    [Header("Дистанция")]
    public float distanceRewardMultiplier = 0.05f;
    public float negativeDistanceRewardMultiplier = -0.01f;
    [Header("Поворот к цели")]
    public float orientationRewardMultiplier = 0.0005f;
    public float negativeOrientationRewardMultiplier = -0.001f;
    [Header("Достижение цели")]
    public float targetReward = 10f;
        [Header("Соприкосновение с землей")]
    [Header(" ")]
    [Header("Соприкосновение с землей 3 ноги (разные)")]
    public float stepRewardThreeDiffernt = 0.05f;
    [Header("Соприкосновение с землей 3 ноги (одинаковые)")]
    public float stepRewardThreeSame = -0.05f;
    [Header("Соприкосновение с землей 4 ноги")]
    public float stepRewardFour = 0.0001f;
    [Header("Соприкосновение с землей < 3 ног")]
    public float stepReward = -0.1f;

    float prevDistance;

    bool reached;

    //private Oscillator m_Oscillator;

    public override void Initialize()
    {
        distToTarget = Vector3.Distance(body.transform.position, cube.transform.position);
        defRot = body.transform.rotation;
        defPos = body.transform.position;

        //m_Oscillator = GetComponent<Oscillator>(); ***
        //m_Oscillator.ManagedReset(); ***
    }

    public void ResetDog()
    {
        Quaternion newRot = Quaternion.Euler(-90, 0, Random.Range(0f, 360f));


        body.TeleportRoot(defPos, newRot);
        //body.TeleportRoot(defPos, defRot); ***
        body.velocity = Vector3.zero;
        body.angularVelocity = Vector3.zero;

        for (int i = 0; i < 12; i++)
        {
            //MoveLeg(legs[i], Random.Range(legs[i].xDrive.lowerLimit, legs[i].xDrive.upperLimit));
            MoveLeg(legs[i], 0);
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        Debug.Log("Heuristic");
    }

    public override void OnEpisodeBegin()
    {
        ResetDog();
        //m_Oscillator.ManagedReset(); ***

        //cube.transform.position = new Vector3(5, 0.21f, Random.Range(-2f, 2f));
        cube.transform.position = new Vector3(Random.Range(-7.5f, 7.5f), 0.21f, Random.Range(-7.5f, 7.5f));
        prevDistance = Vector3.Distance(body.transform.position, cube.transform.position);
        reached = false;
        //cube.transform.position = new Vector3(5f, 0.21f, 0); ***

        //cube.transform.position = new Vector3(8f, 0.26f, 0f);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(body.transform.position);
        sensor.AddObservation(body.velocity);
        sensor.AddObservation(body.angularVelocity);
        sensor.AddObservation(body.transform.right);

        // Позиция куба
        sensor.AddObservation(cube.transform.position);

        // Относительное положение куба
        Vector3 relativePosition = cube.transform.position - body.transform.position;
        sensor.AddObservation(relativePosition);

        // Угловая позиция куба
        Vector3 toCube = (cube.transform.position - body.transform.position).normalized;
        float angleToCube = Vector3.SignedAngle(body.transform.right, toCube, Vector3.up);
        sensor.AddObservation(angleToCube);

        // Расстояние до куба
        float distanceToCube = Vector3.Distance(body.transform.position, cube.transform.position);
        sensor.AddObservation(distanceToCube);
        foreach (var leg in legs)
        {
            sensor.AddObservation(leg.xDrive.target);
            sensor.AddObservation(leg.velocity);
            sensor.AddObservation(leg.angularVelocity);
        }

        foreach(var groundContact in groundContacts)
        {
            sensor.AddObservation(groundContact.touchingGround);
        }
    }

    public void SetRewardForRot()
    {
        if (body.transform.forward.y > 0.95f)
        {
            AddReward(standingReward);
        }
        else
        {
            AddReward(Mathf.Abs(body.transform.forward.y - 0.95f) * notStandingReward);
        }
    }

    private bool RobotHasFallen()
    {
        if (body.transform.forward.y < fallThreshold)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    public void RewardForNotFalling()
    {
        if (!RobotHasFallen())
        {
            AddReward(notFallingRewardMultiplier);
        }
        else
        {
            AddReward(fallingReward);
            Debug.Log("End");
            EndEpisode();
        }
    }

    public override void OnActionReceived(ActionBuffers vectorAction)
    {
        var actions = vectorAction.ContinuousActions;
        float effort = 0.0f;
        for (int i = 0; i < 12; i++)
        {
            float angle = Mathf.Lerp(legs[i].xDrive.lowerLimit, legs[i].xDrive.upperLimit, (actions[i] + 1) * 0.5f);
            float a = Mathf.Clamp(actions[i], -1f, 1f);
            effort += a*a;
            MoveLeg(legs[i], angle);
        }

        //m_Oscillator.ManagedUpdate(); ***

        float currentDistanceToTarget = Vector3.Distance(body.transform.position, cube.transform.position);

        if (!reached && currentDistanceToTarget <= 0.5f) {
            reached=true;
            AddReward(100f);
            EndEpisode();
            return;
        }

        float dt = Time.fixedDeltaTime;

        float progress = prevDistance - currentDistanceToTarget;

        // AddReward(...); - добавление награды
        AddReward(-effort*energyPenalty*dt);
        // AddReward(-currentDistanceToTarget);
        RewardForNotFalling();
        SetRewardForRot();
        AddReward(0.1f*progress);
        AddReward(-0.1f * dt);

        Vector3 toTarget = (cube.transform.position - body.transform.position).normalized;

        float alignment = Vector3.Dot(body.transform.forward, toTarget);
        float facingReward = (alignment + 1f) * 0.5f;
        AddReward(facingReward * 0.00001f);

        prevDistance = currentDistanceToTarget;
        
    }
    public void FixedUpdate()
    {
        body.AddForce((cube.transform.position - body.transform.position).normalized * strenghtMove);
        for (int i = 0; i < 12; i++)
        {
            legs[i].AddForce((cube.transform.position - body.transform.position).normalized * strenghtMove / 20f);
        }

        RaycastHit hit;
        if (Physics.Raycast(body.transform.position, body.transform.right, out hit))
        {
            if (hit.collider.gameObject == cube)
            {
                body.AddForce(2f * strenghtMove * (cube.transform.position - body.transform.position).normalized);
                for (int i = 0; i < 12; i++)
                {
                    legs[i].AddForce((cube.transform.position - body.transform.position).normalized * strenghtMove / 10f);
                }
            }
        }
        Debug.DrawRay(body.transform.position, body.transform.right, Color.white);
    }

    void MoveLeg(ArticulationBody leg, float targetAngle)
    {
        leg.GetComponent<Leg>().MoveLeg(targetAngle, servoSpeed);
    }
}
