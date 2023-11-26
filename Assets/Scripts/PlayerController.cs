using UnityEngine;

public class PlayerController : MonoBehaviour
{
    //referencia al rigidbody
    private Rigidbody _rb;

    [Header("Movement")]
    //fuerza de inpulso por cada vez que se pulsa la tecla
    public float force;
    //limitador de velocidad máxima
    public float maxSpeed;

    [Header("Rotations")]
    //rotacion en Y por cad vez que se pulsa la tecla
    public float yRotation = 20;
    //velocidad del smooth de la rotacion
    public float smoothRotation = 0.2f;

    //angulo objetivo de rotacion del player
    private float _targetYAngle;
    //variable necesaria para el smooth de la rotacion
    private Vector3 _rotateSoothVelocity;

    [Header("Jump")]
    //fuerza de salto
    public float jumpForce;
    public float jumpVelocityDecreaseFactor;

    private float _currentJumpVelocityDecreaseFactor;

    [Header("Gravity")]
    //gravedad a aplicar cuando el objeto salta
    public float gravityForce = 9.8f;

    [Header("Check Ground")]
    //tamaño del raycast
    public float rayLeght;
    //offset del origen del raycast
    public Vector3 rayOffset;
    //layer de deteccion del raycast
    public LayerMask groundLayer;

    //detecta si el jugador está en el suelo
    [SerializeField] private bool _isGrounded;

    void Start() {
        //recuperamos el material del rigidbody
        _rb = GetComponent<Rigidbody>();
    }

    void FixedUpdate() {
        //Llamada de los inputs
        CheckInputs();
        Rotate();
        CheckGround();

        //si no esta en el suelo...
        if(!_isGrounded) {
            //aplicamos la gravedad
            ApplyGravity();
        //si no...
        } else {
            //permitimos al jugador saltar
            Jump();
        }
    }

    /// <summary>
    /// Metodo que chequea los inputs
    /// </summary>
    public void CheckInputs() {
        //la rotacion en y oscilará entre -20, 0 o 20, en funcion de la tecla que estamos presionando,
        //además de aplicar la fuerza de movimiento cada vez que se presione.
        if(Input.GetButtonDown("RightImpulse")) {
            Movement();
            yRotation = 20;
        } else if(Input.GetButtonDown("LeftImpulse")) {
            Movement();
            yRotation = -20;
        } else {
            yRotation = 0;
        }
        if (Input.GetButton("Float")) {
            _currentJumpVelocityDecreaseFactor = jumpVelocityDecreaseFactor;
        }
        else {
            _currentJumpVelocityDecreaseFactor = 1;
        }
    }

    /// <summary>
    /// Método que controla y clamea el movimiento
    /// </summary>
    public void Movement() {
        //Añadimos la fuerza
        _rb.AddForce(transform.forward * force);
        //limitamos la velocidad segun maxspeed
        _rb.velocity = new Vector3(Mathf.Clamp(_rb.velocity.x, -maxSpeed, maxSpeed),
                                   Mathf.Clamp(_rb.velocity.y, -maxSpeed, maxSpeed),
                                   Mathf.Clamp(_rb.velocity.z, -maxSpeed, maxSpeed));
    }

    /// <summary>
    /// Método que gestiona la rotacuión
    /// </summary>
    public void Rotate() {
        //le damos el valor al angulo de target
        _targetYAngle += yRotation;

        //Interpola suavemente la rotación actual hacia el ángulo objetivo
        float smoothYAngle = Mathf.SmoothDampAngle(transform.eulerAngles.y, _targetYAngle, ref _rotateSoothVelocity.y, smoothRotation);

        //Aplica la rotación al jugador
        transform.rotation = Quaternion.Euler(0f, smoothYAngle, 0f);
    }
    /// <summary>
    /// Método que gestiona el salto
    /// </summary>
    public void Jump() {
        if(Input.GetButtonDown("RightImpulse") && Input.GetButtonDown("LeftImpulse")) {
            _rb.AddForce(transform.up * jumpForce);
        }
    }
    /// <summary>
    /// Metodo que aplica la fuerza de la gravedad
    /// </summary>
    private void ApplyGravity() {
        _rb.AddForce(Physics.gravity * gravityForce, ForceMode.Acceleration);
        if (_rb.velocity.y < 0) {
            _rb.velocity = new Vector3(_rb.velocity.x, _rb.velocity.y / _currentJumpVelocityDecreaseFactor, _rb.velocity.z);
        }
    }

    /// <summary>
    /// Metodo que chequea si el jugador esta tocando el suelo
    /// </summary>
    private void CheckGround() {
        Ray ray = new Ray(transform.position + rayOffset, Vector3.down);
        _isGrounded = Physics.Raycast(ray, out RaycastHit hit, rayLeght, groundLayer);
    }
    private void OnDrawGizmos() {
        Gizmos.color = Color.red;
        Gizmos.DrawLine(transform.position + rayOffset, transform.position + rayOffset + Vector3.down * rayLeght);        
    }
}
