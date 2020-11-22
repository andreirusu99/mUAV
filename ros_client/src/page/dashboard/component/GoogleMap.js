import React from "react";
import GoogleMapReact from "google-map-react";


class GoogleMap extends React.Component {

    constructor(props, context) {
        super(props, context);
    }

    getMapOptions = (maps: Maps) => {

        return {
            streetViewControl: false,
            scaleControl: true,
            fullscreenControl: false,
            styles: [{
                featureType: "poi.business",
                elementType: "labels",
                stylers: [{
                    visibility: "off"
                }]
            }],
            gestureHandling: "greedy",
            disableDoubleClickZoom: true,
            minZoom: 18,
            maxZoom: 20,

            mapTypeControl: false,
            mapTypeId: maps.MapTypeId.HYBRID,

            zoomControl: false,
            rotateControl: false,
            clickableIcons: true
        };
    }

    render() {
        return (
            <div style={{height: '300px', width: '100%'}}>
                <GoogleMapReact
                    bootstrapURLKeys={{key: 'AIzaSyBzmy-3g9TlijoJlmFlfrimrAC9SldtW-I'}}
                    options={this.getMapOptions}
                    center={{
                        lat: this.props.center.lat,
                        lng: this.props.center.lng
                    }}
                    defaultZoom={19}
                    yesIWantToUseGoogleMapApiInternals
                >
                    {this.props.markers}
                </GoogleMapReact>

            </div>
        )
    }
}

export default GoogleMap