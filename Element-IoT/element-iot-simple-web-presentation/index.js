const current = document.querySelector('#current>tbody');
const all = document.querySelector('#all>tbody');
const input = document.getElementById('urlInput');
const startSyncButton = document.getElementById('startSync');
let syncLoop = null;
setEventListeners();

function setEventListeners() {
    startSyncButton.addEventListener('click', () => {
        const inputValue = input.value;
        startSync(inputValue);
    })
}

function startSync(inputValue) {
    stopSync();
    syncLoop = setInterval(() => {
	getData(inputValue, setData)
        console.log(inputValue)
    }, 1000)
}

function stopSync(error) {
    if (syncLoop) {
        clearInterval(syncLoop)
    }

    if (error) {
        alert(error)
    }
}

function setData(responseData) {
    responseData = JSON.parse(responseData);
    if(handleResponseErrors(responseData)) return;
    responseData = responseData.body

    all.innerHTML = '';

    current.innerHTML = createTableDataHtml(responseData[0]);
    responseData.shift();

    responseData.forEach(data => {
        all.innerHTML += createTableDataHtml(data)
    });

}

function createTableDataHtml(data) {
    const date = new Date(data.measured_at).toUTCString()
    return `<tr><td data-state="${data.data.contact_state}">${data.data.contact_state}</td><td>${data.data.battery} V</td><td>${date}</td></tr>`
}

function getData(url, callback) {
    const httpRequest = new XMLHttpRequest();

    httpRequest.onreadystatechange = () => {
        if (httpRequest.readyState === XMLHttpRequest.DONE) {
            if (httpRequest.status === 200) {
                console.log(httpRequest.responseText);
                callback(httpRequest.responseText);
            } else {
                stopSync('The url does not yield usable results');
            }
        }
    };

    httpRequest.onerror = (e) => {
        stopSync();
    };

    httpRequest.open('GET', url);
    httpRequest.setRequestHeader('Content-Type', 'application/json');
    httpRequest.send();
}

function handleResponseErrors(responseData) {
    if (!responseData) {
        stopSync('Response is not JSON');
        return true;
    }

    if (responseData.status !== 200) {
        stopSync('Response is not 200');
        return true;
    }

    if (!responseData.body || responseData.body.length <= 0) {
        stopSync('Response body is empty');
        return true;
    }


    return false;
}
