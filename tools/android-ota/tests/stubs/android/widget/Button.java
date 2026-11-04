package android.widget;

import android.content.Context;
import android.view.View;

public class Button extends View
{
    private CharSequence text;

    public Button(Context context)
    {
    }

    public void setText(CharSequence value)
    {
        text = value;
    }

    public CharSequence getText()
    {
        return text;
    }
}
