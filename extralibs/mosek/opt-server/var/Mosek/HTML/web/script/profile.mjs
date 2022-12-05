//-*-javascript-*-
import * as optapi from './optapi.mjs';
import * as utils from  './utils.mjs';
import * as gui from    './gui.mjs';

function inputGetValueAndDefault(id) {
    let n = document.getElementById(id);
    return [n.value, n.getAttribute('data-default')];
}

async function submitUserChanges(identity) {
    var name  = document.getElementById("field-user-name").value;
    var email = document.getElementById("field-user-email").value;
    var pwd   = document.getElementById("field-user-password").value;
    var npwd1 = document.getElementById("field-user-new-pwd1").value;
    var npwd2 = document.getElementById("field-user-new-pwd2").value;

    if (npwd1.length > 0 && npwd2.length > 0 && npwd1 != npwd2) {
        gui.displayError("New password 1 and 2 are not identical - please try again");
        document.getElementById("field-new-pwd").value  = "";
        document.getElementById("field-new-pwd2").value = "";
        document.getElementById("field-new-pwd").focus();
        return;
    }

    var perms = { };
    // perms['submit'] = document.getElementById('field-user-submitter').checked;
    // perms['use-tokens'] = perms['create-tokens'] = document.getElementById('field-user-accesstoken').checked;

    if (pwd != "") {
        var data = { name:name,
                     email:email,
                     perms:perms,
                     oldpassword:pwd };

        if (npwd1.length > 0) {
            data.password = npwd1;
        }

        try {
            let user = await optapi.updateUser(identity.Userid, data);
            gui.displayMessage("User information was updated");
            document.getElementById("field-user-password").value = "";
            document.getElementById("field-user-new-pwd1").value = "";
            document.getElementById("field-user-new-pwd2").value = "";
        }
        catch (err) {
            console.log(err);
            gui.displayError("Failed to update user information");
        }
    }
    else {
        document.getElementById("field-user-password").focus();
        gui.displayError("Need password to authenticate update");
    }
}

function inputSetDefaultAndValue(id,value) {
    let n = document.getElementById(id);
    if (n.getAttribute('type') == 'checkbox')
        n.checked = value;
    else
        n.value = value
    n.setAttribute('data-default',value);
}

function inputResetToDefault(id) {
    let n = document.getElementById(id);
    if (n.getAttribute('type') == 'checkbox')
        n.checked = n.getAttribute('data-default') == 'true';
    else
        n.value = n.getAttribute('data-default');
}
function resetForm() {
    inputResetToDefault('field-user-id');
    inputResetToDefault('field-user-name');
    inputResetToDefault('field-user-email');
    inputResetToDefault('field-user-submitter');
    inputResetToDefault('field-user-accesstoken');
    inputResetToDefault('field-user-admin');
}

export function initialize(identity) {
    inputSetDefaultAndValue('field-user-id',           identity.Userid);
    inputSetDefaultAndValue('field-user-name',         identity.Name);
    inputSetDefaultAndValue('field-user-email',        identity.Email);
    inputSetDefaultAndValue('field-user-login',        identity.Permissions['login']);
    inputSetDefaultAndValue('field-user-submit',       identity.Permissions['submit']);
    inputSetDefaultAndValue('field-user-api',          identity.Permissions['api']);
    inputSetDefaultAndValue('field-user-create-tokens',identity.Permissions['create-tokens']);
    inputSetDefaultAndValue('field-user-admin',        identity.Permissions['admin']);

    document.getElementById('btn-submit').onclick = async () => { await submitUserChanges(identity); }
    document.getElementById('btn-reset').onclick  = () => { resetForm(); }
}
