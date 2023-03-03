document.addEventListener('contextmenu', event => event.preventDefault());



// Show characteristic point:
var canv = document.getElementById('myCanvas');
var ctx = canv.getContext('2d');

// Begins path:
ctx.beginPath();


// Translate origin to a new point:
ctx.translate(350, 350);


draw_workspace();



// Variables that store previous coordinates:
var x_prev = 0;
var y_prev = 300;

// Variables that store current coordinates:
var x_coord = 0;
var y_coord = 300;

// Vectors that store x and y coordinates received:
var x_points = [];
var y_points = [];







document.addEventListener('DOMContentLoaded', function(event)
{
	//const message_holder = document.getElementById('message');

	/*const button_one = document.getElementById( 'button_one' );
	const button_two = document.getElementById( 'button_two' );
	const button_three = document.getElementById( 'button_three' );
	const button_four = document.getElementById( 'button_four' );
	const button_five = document.getElementById( 'button_five' );
	const button_six = document.getElementById( 'button_six' );*/


	// Show coordinates:
	const show_x_message_holder = document.getElementById('show_x_coordinate');
	const show_y_message_holder = document.getElementById('show_y_coordinate');

	// Show current angles:
	const show_theta_1_message_holder = document.getElementById('show_theta_1');
	const show_theta_2_message_holder = document.getElementById('show_theta_2');

	// Show desired angles:
	const show_theta_1_desired_message_holder = document.getElementById('show_theta_1_desired');
	const show_theta_2_desired_message_holder = document.getElementById('show_theta_2_desired');

	// Show delays:
	const show_delta_t_1_message_holder = document.getElementById('show_delta_t_1');
	const show_delta_t_2_message_holder = document.getElementById('show_delta_t_2');



	// Enter coordinates:
	const send_x_message_holder = document.getElementById('input_x');
	const send_y_message_holder = document.getElementById('input_y');



	





	/*
	[ 'mousedown', 'touchstart' ].forEach( function( e )
	{
		button_one.addEventListener( e, function( event ) { action( '1' ); } );
	} );

	[ 'mouseup', 'touchend' ].forEach( function( e )
	{
		button_one.addEventListener( e, function( event ) { action( '0' ); } );
	} );



	[ 'mousedown', 'touchstart' ].forEach( function( e )
	{
		button_two.addEventListener( e, function( event ) { action( '2' ); } );
	} );

	[ 'mouseup', 'touchend' ].forEach( function( e )
	{
		button_two.addEventListener( e, function( event ) { action( '0' ); } );
	} );



	[ 'mousedown', 'touchstart' ].forEach( function( e )
	{
		button_three.addEventListener( e, function( event ) { action( '3' ); } );
	} );

	[ 'mouseup', 'touchend' ].forEach( function( e )
	{
		button_three.addEventListener( e, function( event ) { action( '0' ); } );
	} );



	[ 'mousedown', 'touchstart' ].forEach( function( e )
	{
		button_four.addEventListener( e, function( event ) { action( '4' ); } );
	} );

	[ 'mouseup', 'touchend' ].forEach( function( e )
	{
		button_four.addEventListener( e, function( event ) { action( '0' ); } );
	} );



	[ 'mousedown', 'touchstart' ].forEach( function( e )
	{
		button_five.addEventListener( e, function( event ) { action( '5' ); } );
	} );

	[ 'mouseup', 'touchend' ].forEach( function( e )
	{
		button_five.addEventListener( e, function( event ) { action( '0' ); } );
	} );



	[ 'mousedown', 'touchstart' ].forEach( function( e )
	{
		button_six.addEventListener( e, function( event ) { action( '6' ); } );
	} );

	[ 'mouseup', 'touchend' ].forEach( function( e )
	{
		button_six.addEventListener( e, function( event ) { action( '0' ); } );
	} );
	*/




	function xhr_listener()
	{
		//console.log( this.responseText );
		//message_holder.innerHTML = this.responseText;

		let json_object = JSON.parse(this.responseText);

		if (json_object['type'] == '1')
		{
			show_x_message_holder.innerHTML = json_object['x'];
			show_y_message_holder.innerHTML = json_object['y'];

			show_theta_1_message_holder.innerHTML = json_object['theta_1'];
			show_theta_2_message_holder.innerHTML = json_object['theta_2'];
		}

		console.log(event.type, event.loaded);
	}

	function action(x)
	{
		/*var xhr = new XMLHttpRequest();
		xhr.addEventListener( 'load', xhr_listener );
		xhr.open( 'GET', '/' + x, true );

		xhr.send();*/

		let xhr = new XMLHttpRequest();

		xhr.timeout = 1000; /* Milliseconds */

		//xhr.addEventListener( 'loadstart', xhr_listener );
		//xhr.addEventListener( 'load', xhr_listener );

		xhr.onload = function()
		{
			if (xhr.readyState === xhr.DONE)
			{
				if (xhr.status === 200)
				{
					/*console.log(xhr.response);
					console.log(xhr.responseText);

					message_holder.innerHTML = xhr.responseText;

					show_x_message_holder.innerHTML = xhr.responseText;
					show_y_message_holder.innerHTML = xhr.responseText;
					*/

					let json_object = JSON.parse(xhr.responseText);

					//console.log(json_object);

					// Shows current coordinates and angles:
					if (json_object['type'] == 1)
					{
						show_x_message_holder.innerHTML = json_object['x'];
						show_y_message_holder.innerHTML = json_object['y'];

						show_theta_1_message_holder.innerHTML = json_object['theta_1'];
						show_theta_2_message_holder.innerHTML = json_object['theta_2'];

						// Stores coordinates in variables:
						x_coord = parseInt(json_object['x']);
						y_coord = parseInt(json_object['y']);

						// Draws a line to received point:
						if ((x_coord != x_prev) || (y_coord != y_prev))
						{
							line(x_coord, y_coord);
						}
						
					}

					// Shows delays
					if (json_object['type'] == 2)
					{
						show_delta_t_1_message_holder.innerHTML = json_object['delta_t_1'];
						show_delta_t_2_message_holder.innerHTML = json_object['delta_t_2'];
					}

					// Shows desired angles:
					if (json_object['type'] == 3)
					{
						show_theta_1_desired_message_holder.innerHTML = json_object['theta_1_desired'];
						show_theta_2_desired_message_holder.innerHTML = json_object['theta_2_desired'];
					}
				}
			}
		};
		
		/*xhr.addEventListener( 'loadend', xhr_listener );
		xhr.addEventListener( 'progress', xhr_listener );
		xhr.addEventListener( 'error', () => { console.error( 'XHR Error' ); } );
		xhr.addEventListener( 'abort', () => { console.error( 'XHR Abort' ); } );
		xhr.ontimeout = () => { console.error( 'XHR Timeout' ); };
		*/

		xhr.open('GET', '/' + x, true);
		xhr.send();
	}

	/* clearInterval( get_7 ); get_7 = null; */
	let get_7 = setInterval(() => {action('7');}, '1000');
	let get_8 = setInterval(() => {action('8');}, '1000');
	let get_9 = setInterval(() => {action('9');}, '5000');
});



// Function for sending signal to start calibration:
function calibrate_button()
{
	var xhttp = new XMLHttpRequest();
	xhttp.open("GET", "/calibrate_button_pressed", true);
	xhttp.send();
}

// Function for sending x coordinates:
function send_x()
{
	var xhttp = new XMLHttpRequest();
	xhttp.open("GET", "/send_x", true);
	xhttp.send();
}





// Line tracing function:
function line(x, y)
{
	// Moves to previous point:
	ctx.moveTo(x_prev, -y_prev);

	// Draws a line from previous point to current point:
	ctx.lineTo(x, -y);

	// Sets previous coordonates to current coordinates:
	x_prev = x;
	y_prev = y;

	// Draws the line:
	ctx.stroke();

	console.log('line function');
}



// Function used for resetting canvas:
function clear_canvas()
{
	// Move back to origin:
	ctx.translate(-350, -350);

	// Clear whole canvas:
	ctx.clearRect(0, 0, canv.width, canv.height);

	// Translate old origin to new origin:
	ctx.translate(350, 350);

	// Draw workspace:
	draw_workspace();
	
	// Move to current coordinates:
	ctx.moveTo(x_coord, -y_coord);

	// Begins a new path:
	ctx.beginPath();

	

	//console.log('clear_canvas function');
}



// Function for drawing worksapce onto canvas:
function draw_workspace()
{
	// Begins a new path:
	ctx. beginPath();

	// Draw inner circle of workspace:
	ctx.arc(0, 0, 150, 0, Math.PI, true);

	// Move to point where outer circle starts:
	ctx.moveTo(300, 0);

	// Draw outer circle of workspace:
	ctx.arc(0, 0, 300, 0, Math.PI, true);

	// Close left half of workspace:
	ctx.lineTo(-150, 0);

	// Close right half of workspace:
	ctx.moveTo(150, 0);
	ctx.lineTo(300, 0);

	// Draw everything from above:
	ctx.stroke();

	//console.log('draw_workspace function');
}