//todo: clean up unuseful code
Callback_RelatedTags = function (data) 
{
    modifyWebpart(document.getElementById('divRelTags'), data);
}

Callback_RelatedContent = function(data)
{
    modifyWebpart( document.getElementById('divRelContent'), data);
}


function modifyWebpart( wpdiv, data)
{
    if (!data || data == null)
        return;
        
    if ( !wpdiv)
        return;

    addCommonContainer();
    
    var titlediv = document.createElement('h3');
    var text = document.createElement('span');
    
    wpdiv.style.display = "block";
    wpdiv.innerHTML = data;

    // get the title from the H3 tag and remove the h3 tag
    var h3 = wpdiv.getElementsByTagName('h3')[0];
    var title = '';
    if ( h3 )
    {
        title = h3.childNodes[0].innerHTML;
        var h3parent = h3.parentNode;
        h3parent.removeChild( h3 );
    }
    text.innerHTML = title;
    titlediv.appendChild(text);

    wpdiv.insertBefore(titlediv, wpdiv.childNodes[0]);
}

// add the common container div that will contain both webparts
function addCommonContainer()
{
    if ( null == document.getElementById('gss_sticky_container_div')) // run only once
    {    
        var divRelTags = document.getElementById('divRelTags');
        var divRelCont = document.getElementById('divRelContent');
        if ( divRelTags==null && divRelCont==null)
            return;
            
        if ( divRelTags )
            divRelTags.className = 'gss_stickywebpart_div';
        if ( divRelCont )
            divRelCont.className = 'gss_stickywebpart_div';
        
        var bottomdiv = null;
        var topdiv = null;
        if ( divRelTags!=null && divRelCont==null)
            bottomdiv = topdiv = divRelTags;
        if ( divRelCont!=null && divRelTags==null)
            bottomdiv = topdiv = divRelCont;
            
        if ( bottomdiv == null )
        {
            // when both webparts are displayed determine if the Related Tags 
            // is on top of the Related Content, or vice versa
            var isTagsOnTop = true;
            var div = divRelTags.previousSibling;
            while ( div != null )
            {
                if ( div.id == divRelCont.id )
                {
                    isTagsOnTop = false;
                    break;            
                }
                div = div.previousSibling;
            }
            topdiv = isTagsOnTop ? divRelTags : divRelCont;
            bottomdiv = isTagsOnTop ? divRelCont : divRelTags;
        }
        
        var containerdiv = document.createElement('div');
        containerdiv.id = 'gss_sticky_container_div';
        
        var parentNode = document.getElementById('thinColumn');
        parentNode.insertBefore( containerdiv, bottomdiv.nextSibling );
        
        bottomdiv.className += ' gss_stickywebpart_bottom_div';

        // add the inner div
        var div = document.createElement('div');
        div.id = 'gss_stickyinner_div';
        containerdiv.appendChild(div);
        
        // add the webparts to the inner div
        if ( topdiv )
            div.appendChild(topdiv);            
        if ( bottomdiv )
            div.appendChild(bottomdiv);
    }
}
