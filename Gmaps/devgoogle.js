
function initMap() {

  const directionsService = new google.maps.DirectionsService();
  const directionsRenderer = new google.maps.DirectionsRenderer();
  const directionsService2 = new google.maps.DirectionsService();
  const directionsRenderer2 = new google.maps.DirectionsRenderer();
  //const gebang = { lat: -7.284553, lng: 112.794708 };
  //const keputih = {lat: -7.290162, lng: 112.796708};
  const map = new google.maps.Map(document.getElementById("map"), {
    center: { lat: -7.284795818226529, lng: 112.7964591992797 },
    zoom: 15,
  });

  //Penanda
 // const marker = new google.maps.Marker({
 //   position: gebang,
  //  map: map,
  //  title: 'Gebang'
  //});

  //const marker2 = new google.maps.Marker({
 //   position: keputih,
 //   map: map,
 //   title: 'Keputih'
  //});

  directionsRenderer.setMap(map);

  calculateAndDisplayRoute(directionsService, directionsRenderer);

  document.getElementById("mode").addEventListener("change", () => {calculateAndDisplayRoute(directionsService, directionsRenderer);
  });

  directionsRenderer.setMap(map);

  const onChangeHandler = function () {
    calculateAndDisplayRoute(directionsService2, directionsRenderer2);
  };

  document.getElementById("start").addEventListener("change", onChangeHandler);
  document.getElementById("end").addEventListener("change", onChangeHandler);
}

function calculateAndDisplayRoute(directionsService2, directionsRenderer2) {
  const selectedMode = document.getElementById("mode").value;

  directionsService2
    .route({
      origin: {
        query: document.getElementById("start").value,
      },
      destination: {
        query: document.getElementById("end").value,
      },
      // Note that Javascript allows us to access the constant
      // using square brackets and a string value as its
      // "property."
      travelMode: google.maps.TravelMode[selectedMode],
    })
    .then((response) => {
      directionsRenderer2.setDirections(response);
    })
    .catch((e) => window.alert("Directions request failed due to " + status));

    function calculateAndDisplayRoute(directionsService, directionsRenderer) {
      directionsService
        .route({
          origin: {
            query: document.getElementById("start").value,
          },
          destination: {
            query: document.getElementById("end").value,
          },
          travelMode: google.maps.TravelMode[selectedMode],
        })
        .then((response) => {
          directionsRenderer.setDirections(response);
        })
        .catch((e) => window.alert("Directions request failed due to " + status));
    }
}


window.initMap = initMap;






